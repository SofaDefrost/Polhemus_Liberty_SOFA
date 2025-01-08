"""
Created on Thu Jun 28 13:49:35 2018
@author: akruszew
"""

import usb1  #libusb1
import time

class PolhemusUSB:
    """
    Small Polhemus sensor library 
    Gets the position (x,y,z) in 'cm' and the orientation (quaternion) of each sensors (up to 4)

    Made for linux with usb1 python library installed. """
    
    class sensor:
        """ The data structure that holds the last sensor informations"""
        def __init__(self):
            self._position=[0]*3
            self._quaternion=[0]*4
            self._time = 0

        def GetLastPosition(self):
            """ the last known position in cm since the last call of UpdateSensors"""
            return self._position
            
        def GetLastQuaternion(self):
            """ the last known quaternion since the last call of UpdateSensors"""
            return self._quaternion
        
        def GetLastUpdateTime(self):
            """ the local time of the last call of UpdateSensors"""
            return self._time
        
    def __init__(self,numberOfSensors):

        # USB access informations 
        self._VENDOR_ID = 0x0F44
        self._PRODUCT_ID = 0xFF20
        self._InEP = 0x88
        self._OutEp = 0x4
        self._INTERFACE = 0
        self._CONFIGURATION = 1
        
        # Find the device
        self._handle = usb1.USBContext().openByVendorIDAndProductID(
            self._VENDOR_ID,
            self._PRODUCT_ID,
            skip_on_error=True,
            )

        if self._handle is None:
            print("Device not found")
        try:
            # configuration of the Polhemus
            self._handle.bulkWrite(self._OutEp,b'U1\r',100) #config: in cm
            self._handle.bulkWrite(self._OutEp,b'O*,2,7,1\r',100) #config: in cm
        except usb1.USBErrorTimeout:
            print('time Out while trying to configure the sensor')
            raise

        # init data structure (4 sensors)
        self.sensors=[]
        for i in range(numberOfSensors):
            self.sensors.append(PolhemusUSB.sensor())            
                   
        
    
    def UpdateSensors(self):
        """ Access the Polhemus device and updates the sensors informations"""
        with self._handle.claimInterface(self._INTERFACE):
            try:
                self._handle.bulkWrite(self._OutEp,b'p',100)
                try: 
                    rawData = self._handle.bulkRead(self._InEP,400,100)
                    rawFrames = rawData.split(b'\r\n') 
                    # at this point each line of rawFrames is a list of data for the i-th sensor. The last line is empty
                    # default data format = [ID+Err, x, y, z, Azimuth, Elevation, Roll]
                    for sensor in self.sensors:
                        tempFrame = rawFrames[self.sensors.index(sensor)].split()
                        sensor._position=[float(x) for x in tempFrame[1:4]]
                        sensor._quaternion=[float(x) for x in tempFrame[4:8]]
                        sensor._time=time.time()
                except usb1.USBErrorTimeout:
                    print('time Out while waiting frame datas')
                    pass
            except usb1.USBErrorTimeout:
                    print('time Out while sending the frame request')
                    pass    