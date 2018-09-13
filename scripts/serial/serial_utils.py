import serial

def initSerial(ports, baudrate, time):
	ser = serial.Serial(ports, baudrate, timeout = time)
	return ser


def getLine(serial):
	line = serial.readline()
	return line

def isLineNotEmpty(line):
	if line == '':
		return False
	else:
		return True