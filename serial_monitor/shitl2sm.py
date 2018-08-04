from multiprocessing.connection import Listener, Client
import threading
import queue

#address = ('localhost', 6000)
temp_addr = '/tmp/serialpipe'

is_reading = False
is_writing = False

close_str = '~~close~~'


class SerialMonitorSocket:

	def __init__(self):
		global is_reading
		if not is_reading and not is_writing:
			is_reading = True

		self.read_bytes = queue.Queue() # thread-safe queue where stuff is dumped
		self.read_msg = b""
		print("Starting the listener")
		#listener = Listener(address, authkey=b'supersecretssi')
		listener = Listener(temp_addr, 'AF_UNIX', authkey=b'supersecretssi')
		print("accepting the connection")
		self.listen_conn = listener.accept()

		print("starting up listener thread")

		listen_thread = threading.Thread(name='serial_monitor_thread', target = self._listen)
		listen_thread.start()

	def _listen(self):
		# constantly polling the socket to see if anything new comes in
		while (True):
			msg = self.listen_conn.recv()
			#print(msg)
			self.read_bytes.put(msg)
			if msg == close_str:
				self.listen_conn.close()
				print("Socket Closed")
				break

	
	def read(self, num_bytes):
		# read a number of bytes from the socket stream
		if len(self.read_msg) - num_bytes <= 0:
			self.read_msg += self.read_bytes.get(True) # blocking
		#print("read_msg now is: ", self.read_msg)
		result = self.read_msg[:num_bytes]
		self.read_msg = self.read_msg[num_bytes:]
		#print("Whats left ", self.read_msg)
		return result


class ShitlSocket:

	def __init__(self):
		global is_writing
		if not is_reading and not is_writing:
			is_writing = True
		self.write_msg = b""
		#self.client_conn = Client(address, authkey=b'supersecretssi')
		self.client_conn = Client(temp_addr, 'AF_UNIX', authkey=b'supersecretssi')

	def write(self, data):
		if data == close_str:
			self.client_conn.send(self.write_msg)
			self.client_conn.send(data)
		else:
			self.write_msg += data
			if len(self.write_msg) > 256:
				self.client_conn.send(self.write_msg)
				self.write_msg = b""


	

	



