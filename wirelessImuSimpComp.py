# Simple complementary filter for Wireless IMU app
# The app is sending IMU and Magno data to a certain IP (PC IP) and a target port
# This script is reading them, and using accelerometer and gyro values computing simple 
import socket
import math

UDP_IP = "192.168.0.95"
UDP_PORT = 5555

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

global roll
roll = 0.0
global pitch
pitch = 0.0
global yaw
yaw = 0.0

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    c = ','
    #print "received message:", data
    #print [pos for pos, char in enumerate(data) if char == c]
    commaInd = [pos for pos, char in enumerate(data) if char == c]
    #print data[commaInd[0]+2]
    #print data[commaInd[4]+2]
    #print data[commaInd[8]+2]

    t  = 0.0
    ax = 0.0
    ay = 0.0
    az = 0.0
    gx = 0.0	
    gy = 0.0
    gz = 0.0
    mx = 0.0	
    my = 0.0
    mz = 0.0
    if len(commaInd)!=12 or (data[commaInd[0]+2]!='3') or (data[commaInd[4]+2]!='4') or (data[commaInd[8]+2]!='5'):
    	print "Corrupted data: one of the acc, gyro, magno is not available."
    else:
	t  = float(data[0:commaInd[0]-1])
    	ax = float(data[commaInd[1]+1:commaInd[2]])	
    	ay = float(data[commaInd[2]+1:commaInd[3]])	
    	az = float(data[commaInd[3]+1:commaInd[4]])
    	gx = float(data[commaInd[5]+1:commaInd[6]])
    	gy = float(data[commaInd[6]+1:commaInd[7]])
    	gz = float(data[commaInd[7]+1:commaInd[8]])
    	mx = float(data[commaInd[9]+1:commaInd[10]])
    	my = float(data[commaInd[10]+1:commaInd[11]])
    	mz = float(data[commaInd[11]+1:])
    # end of if-else	
    # Integrate the gyroscope data -> int(angularSpeed) = angle
    dt = 0.01
    pitch = pitch + gx * dt   # Angle around the X-axis
    roll  = roll - gy * dt    # Angle around the Y-axis
    yaw   = yaw - gz*dt       # Angle around the Y-axis
    # Correction for pitch using acceleormeter. Turning around the X axis results in a vector on the Y-axis
    pitchAcc = math.atan2(ay, az)
    pitch = pitch * 0.98 + pitchAcc * 0.02
    # Correction for roll using accelerometer. Turning around the Y axis results in a vector on the X-axis
    rollAcc = math.atan2(ax, az)
    roll = roll * 0.98 + rollAcc * 0.02

    #print "time:", t
    #print "acc-x:", ax
    #print "acc-y:", ay
    #print "acc-z:", az
    #print "gyro-x:", gx
    #print "gyro-y:", gy
    #print "gyro-z:", gz
    #print "magn-x:", mx
    #print "magn-y:", my
    #print "magn-z:", mz
    print "roll:  ", -roll*180/math.pi, "    pitch:  ", pitch*180/math.pi, "    yaw:  ", yaw*180/math.pi


