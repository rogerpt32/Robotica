from test_NeatoCommands import envia

import serial
import time
import math
import numpy as np
import sys
from multiprocessing import Process, Queue


def goTo(angle, rang):

	global cSensors, laserData
	i = int(angle)
	i = (i - rang/2)
	if (i<0): i = i + 360

	minim = 10000000
	distDetector = 800
	val = 0
	for j in range(0,rang):
	
		value = laserData[i][1]
		if value != 0 and value < minim:
			minim = value
		
		i += 1
		if i == 360: i=0

	if (minim > 800): minim = 800
	val = distDetector-minim
	cSensors = val

	return cSensors

def get_motors():
    """ Ask to the robot for the current state of the motors. """
    msg = envia(ser, 'GetMotors LeftWheel RightWheel').split('\n')
    
    # For better understanding see the neato commands PDF.
    
    L = int(msg[4].split(',')[1])
    R = int(msg[8].split(',')[1])

    return (L, R)

def relativeWalls():
	global sensors, laserData
	sensors = []
	distDetector = 800
	val = 0

	for i in range(0, 5):
		minim = 10000000
	
		for j in range(0,36):
			angle = (i * 36 + j + 270) % 360
			value = laserData[angle][1]

			if value != 0 and value < minim:
				minim = value
		
		if (minim > 800): minim = 800
		val = distDetector-minim
		sensors.append(val)
		

def getWalls():

	global laserData

	msg = envia(ser,'GetLDSScan').split('\n')
	laserData = []
	
	for i in range (0,360):
		distMM = (float(msg[i + 2].split(',')[1]))
		laserData.append([float(i), distMM])
		
	relativeWalls()
	
if __name__ == "__main__":
	# Open the Serial Port.
	global ser
	ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.05)
	envia(ser, 'TestMode On')
	envia(ser, 'PlaySound 1')
	envia(ser,'SetLED LEDRed')
	envia(ser ,'SetMotor RWheelEnable LWheelEnable')
	envia(ser,'SetLDSRotation On')

	global distanciaIniR, distanciaIniL, sensors, distL, distR, S

	#INITIAL CONDITIONS
	distanciaIniL, distanciaIniR = get_motors()
	S = 121.5
	speed = 200	# en mm/s
	distL = 0
	distR = 0
	time.sleep(3)

	#envia(ser, 'SetMotor LWheelDist '+ str(5519.778292) +' RWheelDist ' + str(7046.592322) + ' Speed ' + str(speed))

	LTeoIni = 1000
	RTeoIni = 1000
	temps = 0

	while (temps < 5):
		L, R = get_motors()

		distL = L - distanciaIniL
		distR = R - distanciaIniR
		diferenciLR = distL - distR
		
		laserW = getWalls()
		


		
		if (goTo(0,36) == 0):
			print("OBSTACLE EN EL CAMI, DAVANT LLIURE")
			speed = 300
			LTeo = LTeoIni
			RTeo = RTeoIni
		else:
			if ((sensors[0] + sensors[1]) > (sensors[3] + sensors[4])):
				print("CAS4: ESQUIVAR: GIRAR ESQUERRA")
				LTeo = LTeoIni - 4 * (sensors[1] + sensors[0] ) - sensors[2]
				RTeo = RTeoIni + (sensors[1] + sensors[0] ) + sensors[2]
				speed = 250
			else:
				print("CAS5: ESQUIVAR: GIRAR DRETA")
				RTeo = RTeoIni - 4 * (sensors[3] + sensors[4] ) - sensors[2]
				LTeo = LTeoIni + (sensors[3] + sensors[4] ) + sensors[2]
				speed = 250
	

		envia(ser, 'SetMotor LWheelDist '+ str(LTeo) +' RWheelDist ' + str(RTeo) + ' Speed ' + str(speed))

		time.sleep(0.01)
		temps += 0.1
	

	envia(ser, 'TestMode Off', 0.2)
	envia(ser,'SetLDSRotation Off')
	# Close the Serial Port.
	ser.close()      

	print "Final"