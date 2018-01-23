#importing some things
import numpy as np
import os
import time
import socket
import re

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.1.10', 1115))
client_socket.setblocking(0)
client_socket.settimeout(0.1)

cpr = 500
current_pos_1 = 0
current_pos_2 = 0
base_vel = 1500
dt = 1/500

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
	#print('zeros: ' + str(pos) + ' data: ' + str(data))
	client_socket.send(data.encode())
	return data

def incr_pos(position_increment, current_pos):
	return current_pos + position_increment
	
current_pos = np.zeros((8,1))	
command_motors(client_socket, current_pos)

#setpriority()

start_time = time.time()

j = 0
while (1): #time.time()-start_time<25):
	current_pos_1 = (np.sin((time.time()-start_time)*2)*1000)*(1)
			
	for i in range(len(current_pos)):
		current_pos[i] = np.power(-1,(i+1))*current_pos_1
	command_motors(client_socket, current_pos)
	time.sleep(dt)
	
	#if np.abs(current_pos_1) < 5:
	#	time.sleep(0)
	
	if j%50 == 0:
		print('zeros: ' + str(current_pos))
	j = j+1
	
for i in range(len(current_pos)):
	current_pos = np.zeros(8)
	command_motors(client_socket, current_pos)
