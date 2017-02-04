# leddartechone
Ros Drivers for the LeddartechOne This is a driver for the LeddarOne by Leddatech with a RS232 connection port for ROS. The package is written by lasdasdas and licensed under MIT. It has been tested for ROS kinetik and Indigo. 
You will need the std_msgs and the sensor_msgs packages and the modbus as well. You can install the modbus lib with:

>sudo apt install libmodbus5

>sudo apt install libmodbus-dev

After cloning the package on the catkin_ws/src folder, complile it with

> catkin_make

After installing you just have to roslaunch it with:

>roslaunch leddarone leddarone.launch

Some parameters can be set up in the launchfile:
baudrate for the baudrate
slaveid for the slaveid
port the port of the sensor.
This has to be set up like:
>roslaunch leddarone leddarone.launch port:=9600

Other parameters like oversampling are hardcoded, they are already optimized and not really useful to change for a basic use. There is a brief comment on the source code and you will have to check the documentation to change it correctly.

If you get permission problems with the port, you must give yourself the permissions with

sudo usermod -a -G dialout yourusername
