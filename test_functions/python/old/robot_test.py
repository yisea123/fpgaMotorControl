#importing some things
import numpy as np
import cv2
import os
import time
import socket

def fit_lines(base_img, lines):
    if len(lines):
        slopes = np.zeros((len(lines),1))
        counter = 0
        
        for line in lines:
            for x1,y1,x2,y2 in line:
                if y1 > 150 and y2 > 150:
                    cv2.line(base_img,(x1,y1),(x2,y2),(0,255,0),2)
                    dX = np.zeros((2,1))
                    dX[0] = (x2-x1)
                    dX[1] = (y2-y1)
                    m = dX[1]/dX[0]
                    theta = np.arctan(m)
                    #print(theta)
                    if theta < 0:
                        theta = theta + 3.1415
                    slopes[counter] = theta
                    counter = counter+1
                    
        mean_slope = np.sum(slopes)/np.count_nonzero(slopes)
        
        if np.isnan(mean_slope):
            mean_slope = 0

        x = np.int_(base_img.shape[1]/2+np.cos(mean_slope)*100)
        y = np.int_(np.sin(mean_slope)*100)
        cv2.line(base_img,(np.int_(base_img.shape[1]/2),0),(x,y),(255,0,0),2)
        
    return base_img, mean_slope


def draw_lanes(image_name, load_image_flag = 0, masked = 0, plot = 0, hough = 0):
    
    #read image in
    if load_image_flag:
        img = mpimg.imread(image_name)
    else:
        img = image_name
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if hough:
        #blur image
        kernel_size = 5
        blurred = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

        #canny image
        low_threshold = 50
        high_threshold = 150
        canny = cv2.Canny(blurred, low_threshold, high_threshold)


        #mask edges
        if masked:
            border = 0
            vertices = np.array([[(0,gray.shape[0]),(465, 320), (475, 320), (gray.shape[1],gray.shape[0])]], dtype=np.int32)
            canny_masked = region_of_interest(canny, vertices)
        else:
            canny_masked = canny
        #show masked image

        #below does line detection, currently commented out for higher speed
        lines = cv2.HoughLinesP(canny_masked,rho = 1,theta = 1*np.pi/180,threshold = 20,minLineLength = 10,maxLineGap = 200)
        line_img = np.zeros(img.shape, dtype=np.uint8)
        line_img, mean_slope = fit_lines(line_img, lines)
        output = cv2.addWeighted(line_img, 0.5, img, 0.5, 0)
    else:
        output = img
    
    ret,thresh = cv2.threshold(gray,55,255,cv2.THRESH_BINARY_INV)
    image_mean = np.nonzero(thresh)
    cropped_mean = image_mean
    x_location = np.mean(cropped_mean,1)
    PID_input = x_location[1] - gray.shape[0]/2
    
    #https://github.com/ivmech/ivPID example controller

    cv2.circle(output,(np.int_(x_location[1]),np.int_(x_location[0]/2)), 63, (0,0,255), -1)

    if plot:
        plt.subplot(2,2,1)
        plt.imshow(canny)
        cv2.imwrite("canny.jpg", canny)
        plt.subplot(2,2,2)
        plt.imshow(canny_masked)
        plt.subplot(2,2,3)
        plt.imshow(line_img)
        plt.subplot(2,2,4)
        plt.imshow(output)
    
    if hough == 0:
	mean_slope = 0
    return output, mean_slope, x_location[1], x_location[0]

h = 480
w = 640

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.1.219', 1111))

#make both motors finish at same time, modify positions to get angle change, modify velocities based on positions?
counts_per_inch = 250
current_pos_l = 0
current_pos_r = 0
base_vel = 1500
dt = 1/30.0

def command_motors(client_socket, l_pos, l_vel, r_pos, r_vel):
	data = 'b'+str(l_pos) + ' ' + str(l_vel) + ' ' + str(-r_pos) + ' ' + str (-r_vel) + 'd'
	client_socket.send(data.encode())
	return data

def incr_pos(position_increment, current_pos):
	return current_pos + position_increment

cap = cv2.VideoCapture(0)

counter = 0
timeArray = []

while(True):
    ret, frame = cap.read()
    counter = counter + 1
      
    frame = cv2.flip(frame,0)
    small = cv2.resize(frame, (w, h)) 
    
    start_time = time.time()
    small_output, mean_slope, x_location, y_location = draw_lanes(small)

    angle_radians = mean_slope
    angle_degrees = mean_slope*180/3.1415
    #vel_l = base_vel*(1.1+np.cos(angle_radians))
    #vel_r = base_vel*(1.1-np.cos(angle_radians))
    vel_l = base_vel*(1.01+2*(x_location-float(w)/2)/float(w))
    vel_r = base_vel*(1.01-2*(x_location-float(w)/2)/float(w))
    if np.isnan(vel_l):
	vel_l = base_vel
    if np.isnan(vel_r):
	vel_r = base_vel
    vel_l = int(np.max(vel_l, 0))
    vel_r = int(np.max(vel_r, 0))

    current_pos_l = int(incr_pos(vel_l * dt, current_pos_l))
    current_pos_r = int(incr_pos(vel_r * dt, current_pos_r))

    command_motors(client_socket, current_pos_l, vel_l, current_pos_r, vel_r)

    if counter%10 == 0:
	print('vel_l: ' + str(vel_l) + ' vel_r: ' + str(vel_r)) 
	print('pos_l: ' + str(current_pos_l) + ' pos_r: ' + str(current_pos_r))
    	print('Line angle: ' + str(angle_degrees) + ' X centroid: ' + str(x_location) + ' Y centroid: ' + str(y_location) +'\n\n')

    end_time = time.time()
    timeArray.append(end_time-start_time)
    
    output = np.zeros((h, w*2, 3), dtype="uint8")
    output[0:h, 0:w] = small
    output[0:h, w:w * 2] = small_output
    # write the flipped frame
    cv2.imshow('frame', output)

    #time.sleep(0.2)

    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# Release everything if job is finished
cap.release()
cv2.destroyAllWindows()

timeArray = np.array(timeArray)
print(1/np.mean(timeArray))
