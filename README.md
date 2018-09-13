# ROS node for getting RFID data from Arduino
## Dependencies
First of all you must copy libraries from libraries folder to your local Arduino directory (if you do not already have them).

### Connection via WiFi
WiFi connection between Arduino and PC implemented through ESP-01 and it's AT firmware. You should have python-httplib on your PC.
Trere are scetch for MFRC522 reader.

### Connection via serial port
For conection via serial you should install pySerial  (if you do not have it, too).
Trere are scetches for MFRC522 reader and for RDM630/6300.

## Start work
1. Build this package for getting rfid data messages in ROS environment.
2. Upload scetch for your RFID-reader type on you Arduino('s).
3. Run node  
with default parameters:  
`rosrun arduino_rfid ros_rfid_serial.py`  
or  
`rosrun arduino_rfid ros_rfid_wifi.py`  
with specified parameters:  
`rosrun arduino_rfid ros_rfid_serial.py --ports /dev/ttyUSB0 /dev/ttyACM0 --baudrate 9600 --rate 2`
or  
`rosrun arduino_rfid ros_rfid_wifi.py --hosts 192.168.10.115 --port 80 --rate 0.5 --timeout 1`  
you can use launch file where parameters are pre-specified:  
`roslaunch arduino_rfid ros_rfid_serial.launch`.
or  
`roslaunch arduino_rfid ros_rfid_wifi.launch`.
4. See published data in the `/rfid_data` topic (name of topic and node can be changed in `ros_rfid.py`).
5. Enjoy!