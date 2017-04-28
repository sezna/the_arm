
#!/usr/bin/python
#
#	This program  reads the angles from the acceleromter, gyrscope
#	and mangnetometeron a BerryIMU connected to a Raspberry Pi.
#
#	This program includes a number of calculations to improve the 
#	values returned from BerryIMU. If this is new to you, it 
#	may be worthwhile first to look at berryIMU-simple.py, which 
#	has a much more simplified version of code which would be easier
#	to read.   
#
#
#	http://ozzmaker.com/
#
#    Copyright (C) 2016  Mark Williams
#    This library is free software; you can redistribute it and/or
#    modify it under the terms of the GNU Library General Public
#    License as published by the Free Software Foundation; either
#    version 2 of the License, or (at your option) any later version.
#    This library is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
#    Library General Public License for more details.
#    You should have received a copy of the GNU Library General Public
#    License along with this library; if not, write to the Free
#    Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
#    MA 02111-1307, USA



import smbus
import time
import math
from LSM9DS0 import *
import datetime
bus = smbus.SMBus(1)

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40      # Complementary filter constant

#Kalman filter variables
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0



def kalmanFilterY ( accAngle, gyroRate, DT):
	y=0.0
	S=0.0

	global KFangleY
	global Q_angle
	global Q_gyro
	global y_bias
	global YP_00
	global YP_01
	global YP_10
	global YP_11

	KFangleY = KFangleY + DT * (gyroRate - y_bias)

	YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
	YP_01 = YP_01 + ( - DT * YP_11 )
	YP_10 = YP_10 + ( - DT * YP_11 )
	YP_11 = YP_11 + ( + Q_gyro * DT )

	y = accAngle - KFangleY
	S = YP_00 + R_angle
	K_0 = YP_00 / S
	K_1 = YP_10 / S

	KFangleY = KFangleY + ( K_0 * y )
	y_bias = y_bias + ( K_1 * y )

	YP_00 = YP_00 - ( K_0 * YP_00 )
	YP_01 = YP_01 - ( K_0 * YP_01 )
	YP_10 = YP_10 - ( K_1 * YP_00 )
	YP_11 = YP_11 - ( K_1 * YP_01 )

	return KFangleY

def kalmanFilterX ( accAngle, gyroRate, DT):
	x=0.0
	S=0.0

	global KFangleX
	global Q_angle
	global Q_gyro
	global x_bias
	global XP_00
	global XP_01
	global XP_10
	global XP_11


	KFangleX = KFangleX + DT * (gyroRate - x_bias)

	XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
	XP_01 = XP_01 + ( - DT * XP_11 )
	XP_10 = XP_10 + ( - DT * XP_11 )
	XP_11 = XP_11 + ( + Q_gyro * DT )

	x = accAngle - KFangleX
	S = XP_00 + R_angle
	K_0 = XP_00 / S
	K_1 = XP_10 / S

	KFangleX = KFangleX + ( K_0 * x )
	x_bias = x_bias + ( K_1 * x )

	XP_00 = XP_00 - ( K_0 * XP_00 )
	XP_01 = XP_01 - ( K_0 * XP_01 )
	XP_10 = XP_10 - ( K_1 * XP_00 )
	XP_11 = XP_11 - ( K_1 * XP_01 )

	return KFangleX

def writeACC(register,value):
	bus.write_byte_data(ACC_ADDRESS , register, value)
	return -1

def writeMAG(register,value):
	bus.write_byte_data(MAG_ADDRESS, register, value)
	return -1

def writeGRY(register,value):
	bus.write_byte_data(GYR_ADDRESS, register, value)
	return -1



def readACCx():
	acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_X_L_A)
	acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_X_H_A)
	acc_combined = (acc_l | acc_h <<8)

	return acc_combined  if acc_combined < 32768 else acc_combined - 65536


def readACCy():
	acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Y_L_A)
	acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Y_H_A)
	acc_combined = (acc_l | acc_h <<8)

	return acc_combined  if acc_combined < 32768 else acc_combined - 65536


def readACCz():
	acc_l = bus.read_byte_data(ACC_ADDRESS, OUT_Z_L_A)
	acc_h = bus.read_byte_data(ACC_ADDRESS, OUT_Z_H_A)
	acc_combined = (acc_l | acc_h <<8)

	return acc_combined  if acc_combined < 32768 else acc_combined - 65536


def readMAGx():
	mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_X_L_M)
	mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_X_H_M)
	mag_combined = (mag_l | mag_h <<8)

	return mag_combined  if mag_combined < 32768 else mag_combined - 65536


def readMAGy():
	mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Y_L_M)
	mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Y_H_M)
	mag_combined = (mag_l | mag_h <<8)

	return mag_combined  if mag_combined < 32768 else mag_combined - 65536


def readMAGz():
	mag_l = bus.read_byte_data(MAG_ADDRESS, OUT_Z_L_M)
	mag_h = bus.read_byte_data(MAG_ADDRESS, OUT_Z_H_M)
	mag_combined = (mag_l | mag_h <<8)

	return mag_combined  if mag_combined < 32768 else mag_combined - 65536



def readGYRx():
	gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_X_L_G)
	gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_X_H_G)
	gyr_combined = (gyr_l | gyr_h <<8)

	return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536


def readGYRy():
	gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Y_L_G)
	gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Y_H_G)
	gyr_combined = (gyr_l | gyr_h <<8)

	return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536

def readGYRz():
	gyr_l = bus.read_byte_data(GYR_ADDRESS, OUT_Z_L_G)
	gyr_h = bus.read_byte_data(GYR_ADDRESS, OUT_Z_H_G)
	gyr_combined = (gyr_l | gyr_h <<8)

	return gyr_combined  if gyr_combined < 32768 else gyr_combined - 65536




#initialise the accelerometer
writeACC(CTRL_REG1_XM, 0b01100111) #z,y,x axis enabled, continuos update,  100Hz data rate
writeACC(CTRL_REG2_XM, 0b00100000) #+/- 16G full scale

#initialise the magnetometer
writeMAG(CTRL_REG5_XM, 0b11110000) #Temp enable, M data rate = 50Hz
writeMAG(CTRL_REG6_XM, 0b01100000) #+/-12gauss
writeMAG(CTRL_REG7_XM, 0b00000000) #Continuous-conversion mode

#initialise the gyroscope
writeGRY(CTRL_REG1_G, 0b00001111) #Normal power mode, all axes enabled
writeGRY(CTRL_REG4_G, 0b00110000) #Continuos update, 2000 dps full scale

gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
CFangleX = 0.0
CFangleY = 0.0
kalmanX = 0.0
kalmanY = 0.0

a = datetime.datetime.now()

 ## ADS1115 CODE ## 

# Simple demo of reading each analog input from the ADS1x15 and printing it to
# the screen.
# Author: Tony DiCola
# License: Public Domain
import time

# Import the ADS1x15 module.
import Adafruit_ADS1x15


# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()


# Note you can change the I2C address from its default (0x48), and/or the I2C
# bus by passing in these optional parameters:
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 1

 ## MY CODE BEGINS ## 


from matplotlib import pyplot
import numpy as np
import math
import pylab
from mpl_toolkits.mplot3d import Axes3D
import random


fig = pylab.figure()
ax = Axes3D(fig)
def circumference_as_points(radius, center_x, center_y, z, sides=20):
	xs = []
	ys = []
	for angle in range(1, sides + 1):
		print("appending: ", (center_x + radius * math.cos ( 2 / angle )))
		xs.append(center_x + radius * math.cos ( 360 / angle ))
		ys.append(center_y + radius * math.sin ( 360 / angle ))
	return xs, ys


# First, define a starting "home point" that will be modified by the BerryIMU.


def get_second_point(initial_x, initial_y, initial_z, theta_1, theta_2, length):
	return initial_x + (length * math.cos(theta_1)), initial_y + (length * math.sin(theta_2)), initial_z 


pyplot.ion()
# main loop
for i in range(0, 100): # change to while true in prod
	# read ADC values
	wrist = adc.read_adc(1, gain=GAIN)
	elbow = adc.read_adc(3, gain=GAIN)
	y = np.random.random()
	#Read the accelerometer,gyroscope and magnetometer values
	ACCx = readACCx()
	ACCy = readACCy()
	ACCz = readACCz()
	GYRx = readGYRx()
	GYRy = readGYRy()
	GYRz = readGYRz()
	MAGx = readMAGx()
	MAGy = readMAGy()
	MAGz = readMAGz()

	##Calculate loop Period(LP). How long between Gyro Reads
	b = datetime.datetime.now() - a
	a = datetime.datetime.now()
	LP = b.microseconds/(1000000*1.0)
	print "Loop Time | %5.2f|" % ( LP ),


	#Convert Gyro raw to degrees per second
	rate_gyr_x =  GYRx * G_GAIN
	rate_gyr_y =  GYRy * G_GAIN
	rate_gyr_z =  GYRz * G_GAIN


	#Calculate the angles from the gyro. 
	gyroXangle+=rate_gyr_x*LP
	gyroYangle+=rate_gyr_y*LP
	gyroZangle+=rate_gyr_z*LP


	##Convert Accelerometer values to degrees
	AccXangle =  (math.atan2(ACCy,ACCz)+M_PI)*RAD_TO_DEG
	AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG


	####################################################################
	######################Correct rotation value########################
	####################################################################
	#Change the rotation value of the accelerometer to -/+ 180 and
		#move the Y axis '0' point to up.
		#
		#Two different pieces of code are used depending on how your IMU is mounted.
	#If IMU is up the correct way, Skull logo is facing down, Use these lines
	AccXangle -= 180.0
	if AccYangle > 90:
		AccYangle -= 270.0
	else:
		AccYangle += 90.0
	#
	#
	#
	#
	#If IMU is upside down E.g Skull logo is facing up;
	#if AccXangle >180:
		#        AccXangle -= 360.0
	#AccYangle-=90
	#if (AccYangle >180):
		#        AccYangle -= 360.0
	############################ END ##################################


	#Complementary filter used to combine the accelerometer and gyro values.
	CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
	CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

	#Kalman filter used to combine the accelerometer and gyro values.
	kalmanY = kalmanFilterY(AccYangle, rate_gyr_y,LP)
	kalmanX = kalmanFilterX(AccXangle, rate_gyr_x,LP)


	####################################################################
	############################MAG direction ##########################
	####################################################################
	#If IMU is upside down, then use this line.  It isnt needed if the
	# IMU is the correct way up
	#MAGy = -MAGy
	#
	############################ END ##################################


	#Calculate heading
	heading = 180 * math.atan2(MAGy,MAGx)/M_PI

	#Only have our heading between 0 and 360
	if heading < 0:
		heading += 360


	#Normalize accelerometer raw values.
	accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
	accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

	####################################################################
	###################Calculate pitch and roll#########################
	####################################################################
	#Use these two lines when the IMU is up the right way. Skull logo is facing down
	pitch = math.asin(accXnorm)
	roll = -math.asin(accYnorm/math.cos(pitch))
	#
	#Us these four lines when the IMU is upside down. Skull logo is facing up
	#accXnorm = -accXnorm				#flip Xnorm as the IMU is upside down
	#accYnorm = -accYnorm				#flip Ynorm as the IMU is upside down
	#pitch = math.asin(accXnorm)
	#roll = math.asin(accYnorm/math.cos(pitch))
	#
	############################ END ##################################

	#Calculate the new tilt compensated values
	magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
	magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)

	#Calculate tilt compensated heading
	tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

	if tiltCompensatedHeading < 0:
		tiltCompensatedHeading += 360



	if 0:			#Change to '0' to stop showing the angles from the accelerometer
		print ("\033[1;34;40mACCX Angle %5.2f ACCY Angle %5.2f  \033[0m  " % (AccXangle, AccYangle)),

	if 1:			#Change to '0' to stop  showing the angles from the gyro
		print ("\033[1;31;40m\tGRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f" % (gyroXangle,gyroYangle,gyroZangle)),

	if 0:			#Change to '0' to stop  showing the angles from the complementary filter
		print ("\033[1;35;40m   \tCFangleX Angle %5.2f \033[1;36;40m  CFangleY Angle %5.2f \33[1;32;40m" % (CFangleX,CFangleY)),

	if 1:			#Change to '0' to stop  showing the heading
		print ("HEADING  %5.2f \33[1;37;40m tiltCompensatedHeading %5.2f" % (heading,tiltCompensatedHeading)),

	if 0:			#Change to '0' to stop  showing the angles from the Kalman filter
		print ("\033[1;31;40m kalmanX %5.2f  \033[1;35;40m kalmanY %5.2f  " % (kalmanX,kalmanY)),

	#print a new line
	print ""   


	#slow program down a bit, makes the output more readable
	time.sleep(0.03)


	xs = [0]
	ys = [0]
	zs = [0]
	
	bicep_length = 10


	shoulder_theta_x = kalmanX 
	shoulder_theta_y = kalmanY
	elbow_theta    = elbow
	wrist_theta    = wrist
	elbow_x, elbow_y, elbow_z = get_second_point(0, 0, 0, shoulder_theta_x, shoulder_theta_y, bicep_length)
	
	
	xs.append(elbow_x)
	ys.append(elbow_y)
	zs.append(elbow_z)


	for i, line in enumerate(ax.lines):
		ax.lines.pop(i)
#		line.remove()
#	pyplot.scatter(i, y)
	ax.plot(xs, ys, zs)
	pyplot.pause(0.05)

