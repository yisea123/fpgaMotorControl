#importing some things
import numpy as np
import os
import time
import socket
import re

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.1.10', 1111))
client_socket.setblocking(0)
client_socket.settimeout(0.1)

cpr = 500
current_pos_1 = 0
current_pos_2 = 0
base_vel = 1500
dt = 1/100


def setpriority(pid=None,priority=5):
    """ Set The Priority of a Windows Process.  Priority is a value between 0-5 where
        2 is normal priority.  Default sets the priority of the current
        python process but can take any valid process ID. """
        
    import win32api,win32process,win32con
    
    priorityclasses = [win32process.IDLE_PRIORITY_CLASS,
                       win32process.BELOW_NORMAL_PRIORITY_CLASS,
                       win32process.NORMAL_PRIORITY_CLASS,
                       win32process.ABOVE_NORMAL_PRIORITY_CLASS,
                       win32process.HIGH_PRIORITY_CLASS,
                       win32process.REALTIME_PRIORITY_CLASS]
    if pid == None:
        pid = win32api.GetCurrentProcessId()
    handle = win32api.OpenProcess(win32con.PROCESS_ALL_ACCESS, True, pid)
    win32process.SetPriorityClass(handle, priorityclasses[priority])

def command_motors(client_socket, pos):
	data = 'b'+str(int(pos[0])) + ' ' + str(int(pos[1])) + ' ' + str(int(pos[2])) + ' ' + str (int(pos[3])) + ' ' + str (int(pos[4]))  + ' ' + str (int(pos[5])) + ' ' + str (int(pos[6])) + ' ' + str (int(pos[7])) + 'd'
	print('zeros: ' + str(zeroed_axis) + ' data: ' + str(data))
	client_socket.send(data.encode())
	return data

def incr_pos(position_increment, current_pos):
	return current_pos + position_increment

start_time = time.time()

zeroing_rates = [50,10,5,1]
zeroed_axis = np.zeros((8,1))
switches_old = np.zeros((8,1))
current_pos = np.zeros((8,1))
zeroed_position = np.zeros((8,1))
rebumped = np.zeros((8,1))
loop_counter = 0
zero_flag = 1

command_motors(client_socket, current_pos)

print('setting priority')
setpriority()
print('starting loop')
time.sleep(1)
while (1): #time.time()-start_time<25):


	#zeroing routine below
	if (np.sum(zeroed_axis) != 12 or np.min(rebumped[:4]) < 2):
		print('reading switch state')
		switch_state = str(client_socket.recv(256))
		switches = re.split('\s', switch_state)[1:8]
		print(switches)
		print(current_pos)
		print(zeroed_axis)
		print(rebumped)

		for idx, switch in enumerate(switches):
			switch = int(switch)

			if loop_counter == 0:
				switches_old[idx] = switch

			if switches_old[idx] != switch and zeroed_axis[idx] < (len(zeroing_rates)-1):
				zeroed_axis[idx] += 1
			if switch == 0:
				current_pos[idx] += zeroing_rates[int(zeroed_axis[idx])]
			else:
				current_pos[idx] -= zeroing_rates[int(zeroed_axis[idx])]
			if(zeroed_axis[idx] == 3) and switch != switches_old[idx]:
				rebumped[idx] += 1
			switches_old[idx] = switch

		command_motors(client_socket, current_pos)
	else:
		if zero_flag == 1:
			zero_flag = 0
			loop_counter = 0
			start_time = time.time()
			zeroed_position = current_pos.copy()
		current_pos_1 = (np.sin((time.time()-start_time)*2)*8000)*(1)

		for i in range(len(current_pos)):
			current_pos[i] = zeroed_position[i] + np.power(-1,(i+1))*current_pos_1
		command_motors(client_socket, current_pos)
		#now we are zeroed, go to main loop
	loop_counter += 1
	time.sleep(dt)

for i in range(len(current_pos)):
	current_pos[i] = zeroed_position[i]
	command_motors(client_socket, current_pos)