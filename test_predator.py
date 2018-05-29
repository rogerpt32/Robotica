from test_NeatoCommands import envia

import serial
import time
import math
import numpy as np
import sys
from multiprocessing import Process, Queue


def calculaAngleCorreccio(distParet, distParet_anterior, distL):
	delta_dist = abs(distParet - distParet_anterior)
	return (math.atan(delta_dist / distL)) * 360 / (2 * math.pi)

def buscaParet(laserData):
	angle = -1
	dist = 100000
	for i in range(0,360):
		if ((laserData[i][1] < dist) and (laserData[i][1] != 0)):
			angle = i
			dist = laserData[i][1]

	return (angle,dist)

def calculaAngleParet(laserData):
	distanciaEsquerra = goTo(45,36)
	distanciaDreta = goTo(360 - 45, 36)
	if distanciaEsquerra == distanciaDreta: return 0
	elif distanciaDreta < distanciaEsquerra:
		radians = math.atan(distanciaEsquerra/distanciaDreta)
		graus = radians * 360 / (2 * math.pi)
		return graus
	elif distanciaDreta > distanciaEsquerra:
		radians = math.atan(distanciaDreta/distanciaEsquerra)
		graus = radians * 360 / (2 * math.pi)
		return -graus
		

def goTo(angle, rang):

	global cSensors, laserData
	i = int(angle)
	i = (i - rang/2)
	if (i<0): i = i + 360

	minim = 10000000
	val = 0
	for j in range(0,rang):
	
		value = laserData[i][1]
		if value != 0 and value < minim:
			minim = value
		
		i = (i + 1) % 360

	if(minim == 10000000):
		cSensors = -1
		print("CAS RARO")


	cSensors = minim
	return cSensors

def get_motors():
    msg = envia(ser, 'GetMotors LeftWheel RightWheel').split('\n')
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

	MODE = "BUSCANT"
	distParet_anterior = 10000
	distParet = 10000
	EXTRA_SLEEP = "NO"
	
	while (temps < 5):
		L, R = get_motors()

		distL = L - distanciaIniL
		distR = R - distanciaIniR
		diferenciLR = distL - distR
		angle = ((diferenciLR / (2 * S)) * 360 / (2 * math.pi))%360
		if (angle < 0): angle = 360 + angle
		laserW = getWalls()
		print(MODE)

		if MODE == "BUSCANT":
			angleParet, distParet = buscaParet(laserData)
			if angleParet == -1:
				LTeo = 300
				RTeo = 300
				Speed = 200
			else:
				MODE = "APROXIMANT"
				if angleParet < 180:
					gir = (angleParet * 2 * math.pi) / 360 * (2 * S)
					LTeo = -gir/2
					RTeo = gir/2
					speed = 200
				else:
					angleParet = 360 - angleParet
					gir = (angleParet * 2 * math.pi) / 360 * (2 * S)
					LTeo = gir/2
					RTeo = -gir/2
					speed = 200

		elif MODE == "APROXIMANT":
			distParet = goTo(0,5)
			
			if distParet < 300:
				LTeo = 0
				RTeo = 0
				speed = 0
				EXTRA_SLEEP = "SI"
				MODE="BUSCANT"

			else:
				LTeo = distParet * 0.8
				RTeo = distParet * 0.8
				speed = 300
				EXTRA_SLEEP = "SI"
				MODE="BUSCANT"

		envia(ser, 'SetMotor LWheelDist '+ str(LTeo) +' RWheelDist ' + str(RTeo) + ' Speed ' + str(speed))
		if EXTRA_SLEEP == "SI":
			time.sleep(0.75)
			EXTRA_SLEEP = "NO"
	
		time.sleep(0.01)
		temps += 0.01
	

	envia(ser, 'TestMode Off', 0.2)
	envia(ser,'SetLDSRotation Off')
	# Close the Serial Port.
	ser.close()      

	print "Final"