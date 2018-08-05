#from multiprocessing.connection import Listener, Client
import threading
import queue
import sys
import zmq

port = 5556

#address = ('localhost', 6000)
temp_addr = '/tmp/serialpipe'

is_reading = False
is_writing = False

close_bytes = b'~~close~~'

topic = b"ssiserial"


class SerialMonitorSocket:

	def __init__(self):
		global is_reading
		if not is_reading and not is_writing:
			is_reading = True

		self.read_bytes = queue.Queue() # thread-safe queue where stuff is dumped
		self.read_msg = b""

		#listener = Listener(address, authkey=b'supersecretssi')
		#listener = Listener(temp_addr, 'AF_UNIX', authkey=b'supersecretssi')

		listen_thread = threading.Thread(name='serial_monitor_thread', target = self._listen)
		listen_thread.start()



	
	def _listen(self):
		# constantly polling the socket to see if anything new comes in

		# keep the socket in the same thread it's being used in
		context = zmq.Context()
		listener_socket = context.socket(zmq.SUB)
		#listener_socket.connect ("tpc://localhost:%d" % port)
		listener_socket.connect("ipc:///tmp/sm.pipe")
		listener_socket.setsockopt(zmq.SUBSCRIBE, topic)
		
		while(True):
			# 	msg = self.listen_conn.recv() # blocking
			full_msg = listener_socket.recv() # blocking
			msg = full_msg[len(topic)+1:] # add one for the space -> we remove the topic label
			self.read_bytes.put(msg)
			if msg == close_bytes:
			# 	self.listen_conn.close()
				listener_socket.close()
				print("Socket Closed")
				break
	
	def read(self, num_bytes):
		# read a number of bytes from the socket stream
		if len(self.read_msg) - num_bytes <= 0:
			self.read_msg += self.read_bytes.get(True) # blocking
		result = self.read_msg[:num_bytes]
		self.read_msg = self.read_msg[num_bytes:]
		return result


class ShitlSocket:

	def __init__(self):
		global is_writing
		if not is_reading and not is_writing:
			is_writing = True
		self.write_msg = b""
		#self.client_conn = Client(address, authkey=b'supersecretssi')
		#self.client_conn = Client(temp_addr, 'AF_UNIX', authkey=b'supersecretssi')
		context = zmq.Context()
		self.pub_socket = context.socket(zmq.PUB)
		#self.pub_socket.bind("tpc://*:%d" %port)
		self.pub_socket.bind("ipc:///tmp/sm.pipe")

		# http://zguide.zeromq.org/py:syncpub
		#syncservice = context.socket(zmq.REP)
		#syncservice.bind('ipc://*:5562')

	def write(self, data):
		#print(data)
		if data == close_bytes:
			print("sending message: ", self.write_msg)
			self.pub_socket.send(topic + b' ' + self.write_msg)
			self.pub_socket.send(topic + b' ' + data)
			#self.client_conn.send(self.write_msg)
			#self.client_conn.send(data)
		else:
			self.write_msg += data
			if len(self.write_msg) > 256:
				#self.client_conn.send(self.write_msg)
				self.pub_socket.send(topic + b' ' + self.write_msg)
				self.write_msg = b""


	

	



