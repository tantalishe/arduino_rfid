# ROS node for getting RFID data from Arduino
## Dependencies
First of all you must copy libraries from libraries folder to your local Arduino directory (if you do not already have them). Then you should install pySerial  (if you do not have it, too).

## Start work
1. Build this package for getting rfid data messages in ROS environment.
2. Upload scetch for you Arduino('s).
3. Run roscore.
4. Run this node  
with default parameters  
`rosrun arduino_rfid ros_rfid.py`  
with specified ports to connect and baud rate  
`rosrun arduino_rfid ros_rfid.py -p /dev/ttyUSB0 /dev/ttyACM0 -b 9600`  
or  
`rosrun arduino_rfid ros_rfid.py --ports /dev/ttyUSB0 /dev/ttyACM0 --baudrate 9600`  
or you can use launch file where parameters are pre-specified  
`roslaunch arduino_rfid ros_rfid.launch`.
5. See published data in the `/rfid_data` topic (name of topic and node can be changed in `ros_rfid.py`).
6. Enjoy!