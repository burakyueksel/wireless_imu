# Simple complementary filter for Wireless IMU app
# The app is sending IMU and Magno data to a certain IP (PC IP) and a target port
# This script is reading them, and using accelerometer and gyro values computing simple 
import socket
import math
from quaternion import Quaternion
import numpy as np
import warnings
from numpy.linalg import norm

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
global q
q = Quaternion(1, 0, 0, 0)

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    c = ','
    commaInd = [pos for pos, char in enumerate(data) if char == c]
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
    samplePeriod = 1/100
    # q = Quaternion(1, 0, 0, 0)
    beta = 1
    gyroscope = np.array([gx,gy,gz], dtype=float).flatten()
    accelerometer = np.array([ax,ay,az], dtype=float).flatten()
    magnetometer = np.array([mx,my,mz], dtype=float).flatten()
    # Normalise accelerometer measurement
    n_acc = norm(accelerometer)
    if n_acc is 0:
    	warnings.warn("accelerometer is zero")
	n_acc = 0.0001

    accelerometer /= n_acc

    # Normalise magnetometer measurement
    n_magno = norm(magnetometer)
    if n_magno is 0:
    	warnings.warn("magnetometer is zero")
	n_magno = 0.0001

    magnetometer /= n_magno

    h = q * (Quaternion(0, magnetometer[0], magnetometer[1], magnetometer[2]) * q.conj())
    b = np.array([0, norm(h[1:3]), 0, h[3]])

    # Gradient descent algorithm corrective step
    f = np.array([
        2*(q[1]*q[3] - q[0]*q[2]) - accelerometer[0],
        2*(q[0]*q[1] + q[2]*q[3]) - accelerometer[1],
        2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2],
        2*b[1]*(0.5 - q[2]**2 - q[3]**2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]) - magnetometer[0],
        2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]) - magnetometer[1],
        2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]**2 - q[2]**2) - magnetometer[2]
    ])
    j = np.array([
        [-2*q[2],                  2*q[3],                  -2*q[0],                  2*q[1]],
        [2*q[1],                   2*q[0],                  2*q[3],                   2*q[2]],
        [0,                        -4*q[1],                 -4*q[2],                  0],
        [-2*b[3]*q[2],             2*b[3]*q[3],             -4*b[1]*q[2]-2*b[3]*q[0], -4*b[1]*q[3]+2*b[3]*q[1]],
        [-2*b[1]*q[3]+2*b[3]*q[1], 2*b[1]*q[2]+2*b[3]*q[0], 2*b[1]*q[1]+2*b[3]*q[3],  -2*b[1]*q[0]+2*b[3]*q[2]],
        [2*b[1]*q[2],              2*b[1]*q[3]-4*b[3]*q[1], 2*b[1]*q[0]-4*b[3]*q[2],  2*b[1]*q[1]]
    ])
    step = j.T.dot(f)
    step /= norm(step)  # normalise step magnitude

    # Compute rate of change of quaternion
    qdot = (q * Quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])) * 0.5 - beta * step.T

    # Integrate to yield quaternion
    dq = qdot
    dq[0] = dq[0]* samplePeriod
    dq[1] = dq[1]* samplePeriod
    dq[2] = dq[2]* samplePeriod
    dq[1] = dq[3]* samplePeriod
    q = q + dq
    q = Quaternion(q / norm(q))  # normalise quaternion

    pitch = np.arcsin(2 * q[1] * q[2] + 2 * q[0] * q[3])
    if np.abs(q[1] * q[2] + q[3] * q[0] - 0.5) < 1e-8:
        roll = 0
        yaw = 2 * np.arctan2(q[1], q[0])
    elif np.abs(q[1] * q[2] + q[3] * q[0] + 0.5) < 1e-8:
        roll = -2 * np.arctan2(q[1], q[0])
        yaw = 0
    else:
        roll = np.arctan2(2 * q[0] * q[1] - 2 * q[2] * q[3], 1 - 2 * q[1] ** 2 - 2 * q[3] ** 2)
        yaw = np.arctan2(2 * q[0] * q[2] - 2 * q[1] * q[3], 1 - 2 * q[2] ** 2 - 2 * q[3] ** 2)

    print "roll:  ", -roll*180/math.pi, "    pitch:  ", pitch*180/math.pi, "    yaw:  ", yaw*180/math.pi
    #print q[0],"	",q[1],"	",q[2],"	",q[3]
    #print accelerometer[0],"	",accelerometer[1],"	",accelerometer[2]
    #print gyroscope[0],"	",gyroscope[1],"	",gyroscope[2]



