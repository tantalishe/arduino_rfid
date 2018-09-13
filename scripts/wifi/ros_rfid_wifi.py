#!/usr/bin/env python2
import argparse
import rospy
from arduino_rfid.msg import RFID_data
from http_utils import httplibInterface as http

DEFAULT_HOSTS = ["192.168.6.100"] # Hosts that used as default
DEFAULT_RATE = 0.2 # [hz] Rate of publishing
DEFAULT_PORT = 80
DEFAULT_TIMEOUT = 2 # [s] requests timeout

TOPIC_NAME = 'rfid_data'
NODE_NAME = 'serial_rfid_node'

def getArgs():
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--hosts', type=str, nargs='*', dest='hosts', default=DEFAULT_HOSTS, help='list of hosts', required=False)
    parser.add_argument('-p', '--port', required=False, default=DEFAULT_PORT, help='port')
    parser.add_argument('-r', '--rate', type=int, required=False, default=DEFAULT_RATE, help='rate of publishing')
    parser.add_argument('-t', '--timeout', type=int, required=False, default=DEFAULT_TIMEOUT, help='requests timeout')
    args, unknown = parser.parse_known_args()
    return args.hosts, args.port, args.rate, args.timeout

def talker():
	hosts, port, rate, timeout = getArgs()

	# Init ros publisher
	pub = rospy.Publisher(TOPIC_NAME, RFID_data, queue_size=1)
	rospy.init_node(NODE_NAME, anonymous=True)
	rate = rospy.Rate(rate)

	connections = []
	for host in hosts:
			connection = http(host, port, timeout)
			connections.append(connection)

	rospy.loginfo('Working...')
	# For each connection read incoming line,
	# if line not empty, create and publish message
	while not rospy.is_shutdown():
		for connection in connections:
			try:
				line = connection.getResponseLine()
				if http.isLineNotEmpty(line):
					line = int(line)
					msg = RFID_data()
					msg.header.stamp = rospy.Time.now()
					msg.uid = line
					msg.port = connection.host
					pub.publish(msg)
			except  Exception as e:
				rospy.logerr("Exception when getting line from %s", connection.host)
				rospy.logerr(str(e))

		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass