"""
Created on Thu Jun 28 13:49:35 2018
@author: akruszew
Auteurs : Alexandre Kruszewski, Paul Chaillou
Contact : paul.chaillou@inria.fr
Année : 2022
Propriétaire : Université de Lille - CNRS 
License : Non définie
"""
import usb1  #libusb1
import time
from . import quat as qt
from . import utils as utils


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
        
    def __init__(self):
        # USB access informations 
        self._VENDOR_ID = 0x0F44 # en dur ici => faire remonter en argument, pour utilisation de n'importe quel Polhemus ?
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
            print("!!!!!!!!!!! -- Device not found -- !!!!!!!!!!!")
        try:
            # configuration of the Polhemus
            self._handle.bulkWrite(self._OutEp,b'U1\r',100) #config: in cm
            self._handle.bulkWrite(self._OutEp,b'O*,2,7,1\r',100) #config: in cm
        except usb1.USBErrorTimeout:
            print('time Out while trying to configure the sensor')
            raise
        # init data structure (4 sensors)
        self.sensors=[]*4
        for i in range(4):
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
                    for i in range(len(rawFrames)-1):
                        tempFrame = rawFrames[i].split()
                        self.sensors[i]._position=[float(x) for x in tempFrame[1:4]]
                        self.sensors[i]._quaternion=[float(x) for x in tempFrame[4:8]]
                        self.sensors[i]._time=time.time()
                except usb1.USBErrorTimeout:
                    print('time Out while waiting frame datas')
                    pass
            except usb1.USBErrorTimeout:
                    print('time Out while sending the frame request')
                    pass    

class PolhemusOvercoat: # beaucoup de choses en commun avec SofaController => Faire au propre et factoriser ? Ou faire appeler la fonction Overcoat par le ControllerSOFA (c'est mieux que la 1ere option je pense)
    """
    Surcouche logicielle pour faire correspondre les référentiels
    """
    def __init__(self, init_pos = [0,0,0],conversion_factor = 10,axis = [0,1,2],print_flag = False):

        self.sensor = PolhemusUSB()
        self.conv = conversion_factor # = 10 --> convert from cm to mm

        self.print_flag = print_flag

        for i in range(10): # on récupère les positions 10 fois, pour éviter les valeurs fausses à l'initialisation, qui entreneraient un recalage des référentiels faux
            self.sensor.UpdateSensors()
            position = self.sensor.sensors[0].GetLastPosition()
            rotation = self.sensor.sensors[0].GetLastQuaternion()

        self.axis = axis

        self.inversion_tab = [1,1,1]
        for i in range(len(axis)) : # pour définir les axes à inverser en mettant l'indice en négatif dans axis
            if axis[i] < 0 :
                self.axis[i] = abs(axis[i])
                self.inversion_tab[axis[i]] = -1
            # self.axis[i] = abs(axis[i])

        print(self.inversion_tab)

        x_i = position[self.axis[0]]
        y_i = position[self.axis[1]]
        z_i = position[self.axis[2]]

                
        self.initialTranslationOffset = [ init_pos[0]-x_i*self.conv, init_pos[1]-y_i*self.conv, init_pos[2]-z_i*self.conv ] # si l'offset est définie dans le référentiel du Polhemus (pas naturel du tout)

        q = qt.Quat([rotation[1],rotation[2],rotation[3],-rotation[0]]) # Signe négatif sur le dernier élément pour prendre le conjugué ; équivalent à l'inverse pour les quaternion (si q permet de passer du repère a au repère b, q.conj permet de passer de b à a) // Décalage dans les valeurs pour la correspondance des référentiels
      
        self.initialRotationOffset = utils.fit_referential(self.axis, q,self.inversion_tab)

        if self.print_flag == True :
            print('sensors_initial_translation = ' + str(self.initialTranslationOffset) )
            print('sensors_initial_rotation = ' + str(self.initialRotationOffset ) )

    def get_data(self):       
        self.sensor.UpdateSensors()
        position = self.sensor.sensors[0].GetLastPosition()

        # import et conversion des quaternions
        r = self.sensor.sensors[0].GetLastQuaternion()
        q1 = qt.Quat([r[1],r[2],r[3],r[0]])
        # q1 = qt.Quat([r[0],r[1],r[2],r[3]])
        q1 = utils.fit_referential(self.axis, q1,self.inversion_tab)
        q1.rotateFromQuat(self.initialRotationOffset)

        # q2 = alignQuatX(q1) # on annule le roulis du capteur # le faire ici ? PLutôt le mettre dans le CloseLoopController non ? Je trouve ça bizarre de mettre dans un code qui récupère les données du Polhemus et les importe dans SOFA, ce genre de contrainte devrait être visible dans le controller plutôt
        q2 = q1 
        q2 = utils.pansement(self.axis, q1,self.inversion_tab)

        pos_raw = position[:3]
        pos = [pos_raw[self.axis[0]]*self.conv + self.initialTranslationOffset[0], pos_raw[self.axis[1]]*self.conv + self.initialTranslationOffset[1], pos_raw[self.axis[2]]*self.conv + self.initialTranslationOffset[2]]
        
        if self.print_flag == True :
            print("Position before inversion :")  
            print(pos)

        for i in range(3):
            pos[i] = pos[i] * self.inversion_tab[i]
            
        if self.print_flag == True :
            print("Position after inversion :") 
            print(pos)

        return [pos,q2]


class PolhemusOvercoat_Pos: # beaucoup de choses en commun avec SofaController => Faire au propre et factoriser ? Ou faire appeler la fonction Overcoat par le ControllerSOFA (c'est mieux que la 1ere option je pense)
    """
    Surcouche logicielle pour faire correspondre les référentiels
    """
    def __init__(self, init_pos = [0,0,0],conversion_factor = 10,axis = [0,1,2],print_flag = True):

        self.sensor = PolhemusUSB()
        self.conv = conversion_factor # = 10 --> convert from cm to mm

        self.print_flag = print_flag

        for i in range(10): # on récupère les positions 10 fois, pour éviter les valeurs fausses à l'initialisation, qui entreneraient un recalage des référentiels faux
            self.sensor.UpdateSensors()
            # position = self.sensor.sensors[0].GetLastPosition()
            # rotation = self.sensor.sensors[0].GetLastQuaternion()

        self.axis = axis

        self.inversion_tab = [1,1,1]
        for i in range(len(axis)) : # pour définir les axes à inverser en mettant l'indice en négatif dans axis
            if axis[i] < 0 :
                self.axis[i] = abs(axis[i])
                self.inversion_tab[axis[i]] = -1
            # self.axis[i] = abs(axis[i])

        print(self.inversion_tab)

        # x_i = position[self.axis[0]]
        # y_i = position[self.axis[1]]
        # z_i = position[self.axis[2]]

                
        # self.initialTranslationOffset = [  ] # si l'offset est définie dans le référentiel du Polhemus (pas naturel du tout)

        # q = qt.Quat([rotation[1],rotation[2],rotation[3],-rotation[0]]) # Signe négatif sur le dernier élément pour prendre le conjugué ; équivalent à l'inverse pour les quaternion (si q permet de passer du repère a au repère b, q.conj permet de passer de b à a) // Décalage dans les valeurs pour la correspondance des référentiels
      
        # self.initialRotationOffset = utils.fit_referential(self.axis, q,self.inversion_tab)

        self.base_vector =  [ 250, -60,-200]

        # if self.print_flag == True :
        #     print('sensors_initial_translation = ' + str(self.initialTranslationOffset) )
            # print('sensors_initial_rotation = ' + str(self.initialRotationOffset ) )

    def get_data(self):       
        self.sensor.UpdateSensors()
        position = self.sensor.sensors[0].GetLastPosition()

        # # import et conversion des quaternions
        # r = self.sensor.sensors[0].GetLastQuaternion()
        # q1 = qt.Quat([r[1],r[2],r[3],r[0]])
        # # q1 = qt.Quat([r[0],r[1],r[2],r[3]])
        # q1 = utils.fit_referential(self.axis, q1,self.inversion_tab)
        # q1.rotateFromQuat(self.initialRotationOffset)

        # # q2 = alignQuatX(q1) # on annule le roulis du capteur # le faire ici ? PLutôt le mettre dans le CloseLoopController non ? Je trouve ça bizarre de mettre dans un code qui récupère les données du Polhemus et les importe dans SOFA, ce genre de contrainte devrait être visible dans le controller plutôt
        # q2 = q1 
        # q2 = utils.pansement(self.axis, q1,self.inversion_tab)

        pos_raw = position[:3]
        pos = [pos_raw[self.axis[0]]*self.conv, pos_raw[self.axis[1]]*self.conv , pos_raw[self.axis[2]]*self.conv ]
        
        if self.print_flag == True :
            print("Position before inversion :")  
            print(pos)

        for i in range(3):
            pos[i] = pos[i] * self.inversion_tab[i]
            pos[i] = pos[i] + self.base_vector[i]
            
        if self.print_flag == True :
            print("Position after inversion :") 
            print(pos)

        # pos = pos + 


            # Appliquer le passage de base
        return pos

        # return [pos,q2]
        
