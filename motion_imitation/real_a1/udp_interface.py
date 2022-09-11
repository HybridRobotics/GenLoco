import time
import signal
import sys
import numpy as np
import socket
from motion_imitation.robots import a1

def sigint_handler(signum, frame):
	print("  Ctrl + C Exit!")
	sys.exit(0)
signal.signal(signal.SIGINT, sigint_handler) 

class UDP_for_net:
	def __init__(self,
	             recv_IP='127.0.0.1',
	             recv_port=8000,
                 send_IP='127.0.0.1',
                 send_port=8001):
		self.sender_IP = send_IP
		self.UDP_PORT_SENDING = send_port
		self.sending_freq = 1000.0
		self.sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sender_socket.settimeout(0.01)
		self.sender_addr = (self.sender_IP, self.UDP_PORT_SENDING)
		print("UDP Sender is initialized:",send_IP)

		self.receiver_IP = recv_IP
		self.UDP_PORT_RECVING = recv_port
		self.receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.receiver_socket.settimeout(0.01)
		self.receiver_socket.bind((self.receiver_IP, self.UDP_PORT_RECVING))
		print("UDP Receiver is initialized:",recv_IP)

	def send_pack(self,message):
		# print("Sending")
		self.sender_socket.sendto(message, self.sender_addr)

	def receive_wait(self):
		data,_ = self.receiver_socket.recvfrom(1024)  # current buffer size is 1024 bytes
		return data  # return raw msg

class a1_interface:
	def __init__(self,
				 robot_class = a1,
	             recv_IP='192.168.0.34',
	             recv_port=32770,
	             send_IP='192.168.0.1',
	             send_port=32769) -> None:
		self._robot_class =robot_class
		self.udp = UDP_for_net(recv_IP, recv_port, send_IP, send_port)
		self._last_obs = np.concatenate([[1],np.zeros(3),np.zeros(3),np.array(robot_class.INIT_MOTOR_ANGLES*robot_class.JOINT_DIRECTIONS),[0]],axis=-1)
		self._init_obs = np.concatenate([[1],np.zeros(3),np.zeros(3),np.array(robot_class.INIT_MOTOR_ANGLES*robot_class.JOINT_DIRECTIONS),[0]],axis=-1)
		self._udp_init_done = False
		self._start_walking = False
		self._count = 0
		self._init_time = time.time()

	def send_command(self, action):
		if self._udp_init_done is False:
			action = np.ones(12)*-10
		print("action_raw",action)
		action = np.round(action, 5) 
		action = list(map(lambda x: str(x), list(action)))
		action +=" "
		msg = " ".join(action).encode('utf-8')
		self.udp.send_pack(msg) 
			

	def receive_observation(self):  # receive from gazebo and process it to array
		while 1:
			try:
				receive = self.udp.receive_wait()			
				data = receive.split()
				data = list(map(lambda x: float(x), data))
				print("Received data", data)
				print("Is walking?",data[18])
				self._last_obs = data
				if self._start_walking is False:
					if self._udp_init_done is True:
						if data[18] == 1:
							self._start_walking = True
						return data
					else:
						if self._count>=3:
							self._udp_init_done = True
						self._count +=1
						return self._init_obs
				else:
					return data
						
			except socket.timeout:
				print("Receive observation timeout!")
				
				if self._start_walking ==True:
					return self._last_obs
				else:
					return self._init_obs