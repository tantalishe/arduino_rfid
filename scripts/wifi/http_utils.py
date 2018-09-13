import httplib


class httplibInterface:

	def __init__(self, host, port, timeout):
		self.host = host
		self.port = port
		self.timeout = timeout

	def initConnection(self):
		connection = httplib.HTTPConnection(self.host, self.port, timeout = self.timeout)
		return connection

	def getResponseLine(self):
		connection = self.initConnection()
		connection.request("GET", "")
		response = connection.getresponse()
		response_line = response.read()
		connection.close()
		return response_line

	@staticmethod
	def isLineNotEmpty(line):
		if line == '':
			return False
		else:
			return True