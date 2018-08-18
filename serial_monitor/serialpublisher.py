import serial
import queue
import threading
import re
import zmq


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

class SerialReader:
	
	def __init__(self, garbage_fn):
		self.garbage = garbage_fn

		# socket stuff
		context = zmq.Context()
		self.pub_socket = context.socket(zmq.PUB)
		self.pub_socket.bind("ipc:///tmp/ser_pub.pipe")
	
		# start thread to watch read_write_queue
		watch_thread = threading.Thread(name='watch_queue_thread', target = self.watch_queue)
		watch_thread.start()

	def read(self):
		garbage_buff = b''
		while(True):
			print('trying to read')
			byte = teensy.read() # blocking
			print('read something')
			if byte == FSTART:
				print("OH MY GOD I GOT IT")
			if byte in signal_chars:
				si = signal_chars_map[byte]

				# put start char in buffer
				publish_buffer = byte
				# read the time
				publish_buffer += teensy.read(4)
				# read the rest of data
				publish_buffer += teensy.read(4*si.var_count)

				# publish the data read
				print('publishing to socket')
				self.pub_socket.send(si.process_topic + b' ' + publish_buffer)
				print('published to socket')
			else:
				#garbage_buff += byte
				self.garbage(byte)


	def watch_queue(self):
		# check if theres anything in read_write_queue
		while(True):
			#if not read_write_queue.empty():
			code, msg = read_write_queue.get() # blocking
			if code == 1: # change garbage function
				print("watch_queue: changing garbage function")
				self.garbage = msg
			if code == 2: # send fstart to new subscriber, msg is the topic
				print("watch_queue: registering topic")
				self.pub_socket.send(msg + b' ' + FSTART)



class SerialWriter:
	def __init__(self):
		# socket stuff
		context = zmq.Context()
		self.pull_socket = context.socket(zmq.PULL)
		self.pull_socket.bind("ipc:///tmp/ser_pull.pipe")

	def write(self):
		while(True):
			# pull information from socket
			recv_msg = self.pull_socket.recv()
			# check if should write it or edit something
			flag = recv_msg[0]
			msg = recv_msg[1:]

			if flag == b'\x00':
				# write to serial
				teensy.write(msg)
			elif flag == b'\x01':
				# update garbage function
				read_write_queue.put((1, msg))
			elif flag == b'\x02':
				# register new subscriber
				read_write_queue.put((2, msg))


def garbage_print(byte):
	print(byte, end='')

def read_serial():
	serial_reader = SerialReader(garbage_print)
	read_thread = threading.Thread(name='read_thread', target = serial_reader.read)
	read_thread.start()

def write_serial():
	serial_writer = SerialWriter()
	write_thread = threading.Thread(name='write_thread', target = serial_writer.write)

if __name__=='__main__':
	setup()
	read_serial()
	write_serial()

