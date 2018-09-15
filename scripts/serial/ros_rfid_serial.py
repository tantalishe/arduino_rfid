#!/usr/bin/env python2
import argparse
import rospy
from arduino_rfid.msg import RFID_data
from serial_utils import *

DEFAULT_PORTS = ["/dev/ttyUSB0"] # Ports that used as default
BAUDRATE = 9600
RATE = 2 # [hz] Rate of publishing

TOPIC_NAME = 'rfid_data'
NODE_NAME = 'serial_rfid_node'

def getArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--ports', type=str, nargs='*', dest='ports', default=DEFAULT_PORTS, help='list of serial ports', required=False)
    parser.add_argument('-b', '--baudrate', required=False, default=BAUDRATE, help='specifies baudrate')
    parser.add_argument('-r', '--rate', type=float, required=False, default=RATE, help='rate of publishing [hz]')
    args, unknown = parser.parse_known_args()
    return args.ports, args.baudrate, args.rate

def talker():
	ports, baudrate,rate = getArgs()

	# Init ros publisher
	pub = rospy.Publisher(TOPIC_NAME, RFID_data, queue_size=10)
	rospy.init_node(NODE_NAME, anonymous=True)
	rate = rospy.Rate(rate) # 10hz

	# Init serial for each port
	rospy.loginfo('Ports opening...')
	serials = []
	for port in ports:
		try:
			serial = initSerial(port, baudrate,0)
			serials.append(serial)
		except  Exception as e:
			rospy.logwarn(str(e))

	# If all of specified ports cant be opened, the node is closed
	if not serials:
		rospy.logwarn("There are no opened ports")
		return

	rospy.loginfo('Working...')
	# For each opened port read incoming line,
	# if line not empty, create and publish message
	while not rospy.is_shutdown():
		for serial in serials:
			line = getLine(serial)
			if isLineNotEmpty(line):
				try:
					line = int(line)
					msg = RFID_data()
					msg.header.stamp = rospy.Time.now()
					msg.uid = line
					msg.port = serial.port
					pub.publish(msg)
				except ValueError:
					# Incoming data must have int type (uid)
					rospy.logerr("Wrong data format from port %s", serial.port)

		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass