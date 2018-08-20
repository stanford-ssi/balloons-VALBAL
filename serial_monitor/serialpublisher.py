import serial
import queue
import threading
import re
import zmq

DEBUG = False

#teensy = serial.Serial('/dev/ttyACM0',baudrate=115200)
teensy = serial.Serial('/dev/tty.usbmodem14411',baudrate=115200)
FSTART = b'\xaa'

# keeps track of each subscriber's variable name
class Subscriber_Info:
	def __init__(self, start_char, file_name, process_topic, start_comment="diddlybop"):
		self.start_char = start_char
		self.file_name = file_name
		self.process_topic = process_topic
		self.start_comment = start_comment
		self.var_names = []
		self.var_count = 0

sub_list = []
sub_list.append(Subscriber_Info(
		b'\xaa',
		'../src/Shitl.cpp',
		b'shitl'
	)
)
sub_list.append(Subscriber_Info(
		b'\xab',
		'../src/SerialMonitor.cpp',
		b'serial-monitor'
	)
)


signal_chars = [si.start_char for si in sub_list]
signal_chars_map = dict(zip(signal_chars, sub_list))

print(signal_chars)
def setup():
	# read the cpp files and extract the variable names and the number of variables being sent over
	for si in sub_list:
		file = open(si.file_name, 'r')
		lines = file.readlines()
		for i,line in enumerate(lines):
		    if re.search(si.start_comment, line):
		    	si.var_count = int(re.findall(r"\[(.*?)\]",lines[i+1])[0])
		    	si.var_names = [re.findall(r"\= (.*?)\;",k)[0] for k in lines[i+2:i+2+si.var_count]]
		    	print(si.var_names)
		    	break
		si.var_names.insert(0,'time')


################################ READ AND WRITE SERIAL #######################################

# for communication between the reading and writing threads
read_write_queue = queue.Queue()

# for fast sockets maybe?
context = zmq.Context()

class SerialReader:
	
	def __init__(self, garbage_fn):
		self.garbage = garbage_fn
		self.has_listeners = False
	
		self.pub_queue = queue.Queue()
		self.sub_list = []

		# start thread to watch read_write_queue
		watch_thread = threading.Thread(name='watch_queue_thread', target = self.watch_queue)
		watch_thread.start()

		# start thread to watch pub_queue
		publish_thread = threading.Thread(name='publish_thread', target = self.publish)
		publish_thread.start()

	def read(self):
		garbage_buff = b''

		while(True):
			byte = teensy.read() # blocking
			if byte == FSTART:
				print("Received first FSTART")

			if self.has_listeners: # don't want to send the first FSTART...
				if byte in signal_chars:
					si = signal_chars_map[byte]

					# put start char in buffer
					publish_buffer = byte
					# read the time
					publish_buffer += teensy.read(4)
					# read the rest of data
					publish_buffer += teensy.read(4*si.var_count)

					# publish the data read
					if DEBUG: print('serialpublisher read: publishing', publish_buffer,  'to queue with topic', si.process_topic)
					self.pub_queue.put(si.process_topic + b' ' + publish_buffer)
					if DEBUG: print('serialpublisher read: published to socket')
				else:
					# this is where the garbage output is sent
					if DEBUG: print("serialpublisher.py read: garbage stuff")

					if b'serial-monitor' in self.sub_list:
						self.pub_queue.put(b'serial-monitor ' + byte)
					else:
						try:
							print(byte.decode("utf-8"), end="")
						except:
							pass
					#self.garbage(byte)


	def publish(self):
		# socket stuff - in same thread
		pub_socket = context.socket(zmq.PUB)
		pub_socket.bind("ipc:///tmp/ser_pub.pipe")
		#pub_socket.bind("tcp://*:5556")

		while(True):
			msg = self.pub_queue.get()
			pub_socket.send(msg)



	def watch_queue(self):
		
		# check if theres anything in read_write_queue
		while(True):
			#if not read_write_queue.empty():
			code, msg = read_write_queue.get() # blocking
			if code == 1: # change garbage function
				if DEBUG: print("serialpublisher.py watch_queue: changing garbage function")
				self.garbage = msg
			elif code == 2: # send fstart to new subscriber, msg is the topic
				if DEBUG: print("serialpublisher.py watch_queue: registering topic", msg)
				self.sub_list.append(msg)
				self.has_listeners = True
				self.pub_queue.put(msg + b' ' + FSTART)
			elif code == 3:
				if DEBUG: print("serialpublisher.py watch_queue: de-registering topic", msg)
				try:
					self.sub_list.remove(msg)
				except:
					pass





class SerialWriter:
	def __init__(self):
		self.first_write = True

	def write(self):
		# socket stuff - in same thread
		sub_socket = context.socket(zmq.SUB)
		sub_socket.bind("ipc:///tmp/ser_pull.pipe")
		sub_socket.setsockopt(zmq.SUBSCRIBE, b'w') # w for 'write'
		while(True):
			# pull information from socket
			if DEBUG: print("serialpublisher.py write: Waiting for messsage")
			recv_msg = sub_socket.recv()
			# check if should write it or edit something
			if DEBUG: print("serialpublisher.py write: Just read message",recv_msg)
			flag = recv_msg[2] # first byte after 'topic' and space
			msg = recv_msg[3:] # rest of message
			if DEBUG: print("serialpublisher.py write: flag =", flag)
			if DEBUG: print("serialpublisher.py write: msg =", msg)

			if flag == 0x0:
				# write to serial
				if DEBUG: print("serialpublisher.py write: writing to the teensy")

				# check to make sure what you're only writing FSTART if it's the first time you're writing
				if msg != FSTART or self.first_write: # NOT (msg = FSTART and not self.first_write)
					teensy.write(msg)
					self.first_write = False
				else:
					if DEBUG: print("serialpublisher.py write: didn't write to teensy")

			elif flag == 0x1:
				# update garbage function
				read_write_queue.put((1, msg))
			elif flag == 0x2:
				# register new subscriber
				if DEBUG: print("serialpublisher.py write: registering new subscriber")
				read_write_queue.put((2, msg))
			elif flag == 0x3:
				# deregister new subscriber
				if DEBUG: print("serialpublisher.py write: registering new subscriber")
				read_write_queue.put((3, msg))


def garbage_print(byte):
	print(byte, end='')

def read_serial():
	serial_reader = SerialReader(garbage_print)
	read_thread = threading.Thread(name='read_thread', target = serial_reader.read)
	read_thread.start()

def write_serial():
	serial_writer = SerialWriter()
	write_thread = threading.Thread(name='write_thread', target = serial_writer.write)
	write_thread.start()

if __name__=='__main__':
	setup()
	read_serial()
	write_serial()

