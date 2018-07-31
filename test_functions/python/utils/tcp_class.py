import socket


class tcp_communication():
	def __init__(self, ip, port):
		self.ip = ip
		self.port = port

	def open_socket(self):
		#Initialize socket
		print("Opening socket at ip: {} using port: {}".format(self.ip, self.port))
		CLIENT_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		CLIENT_SOCKET.connect((self.ip, self.port))
		CLIENT_SOCKET.setblocking(1)
		#CLIENT_SOCKET.settimeout(1) #if setblocking is 1, this is basically settimeout(None)

		# #Run a single command to initalize communication
		# self.command_motors(position)
		# time.sleep(1)
		return CLIENT_SOCKET

	def setpriority(self, pid=None, priority=5):
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