# Polhemus sensor

## Test

Launch the main_polhemus.py script to test the connexion to the Polhemus. You have to launch it as sudo administrator.

```console
~/polhemus_liberty $ python main_polhemus.py
```

## Test with SOFA

You can import the Polhemus data into Matlab thanks to a SOFA python controller. Launch the SOFA_scene_polhemus.py to test it. You have to enable the display of the collision model to see the orange dot moving in your referential :

```console
~/polhemus_liberty $ runSofa SOFA_scene_polhemus.py 
```

You may also use the orientation in SOFA:

```console
~/polhemus_liberty $ runSofa SOFA_scene_polhemus_orientation.py 
```


## Error

If you have the following error :

```console
~/polhemus_liberty $ sudo python3 main_polhemus.py
    import usb1  #libusb1
ModuleNotFoundError: No module named 'usb1'
```

Then launch the following command :

```console
~/polhemus_liberty $ sudo -E python3 main_polhemus.py
```

This force the sudo mode to keep the python environment.

## Allow usb port

To launch the script, you have to do the following commands to open the usb port. You have to do that every time you reconnect the sensor as te device change.

```console
~ $ lsusb
Bus 001 Device 014: ID 0f44:ff20 Polhemus Liberty 2.0
~ $ lsusb | grep 014 | sed -nr 's|Bus (.*) Device ([^:]*):.*|/dev/bus/usb/\1/\2|p'
/dev/bus/usb/001/014
~ $ sudo chmod 777 /dev/bus/usb/001/014
```
