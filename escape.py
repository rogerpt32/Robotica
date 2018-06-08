#!/usr/bin/python
#coding: utf8

"""
Imports
"""
import time
import math
import serial

"""
Imports de Teclado
"""
import os
import sys, tty, termios
from select import select

# To import the function "envia" from the file "test_NeatoCommands.py"
from test_NeatoCommands import envia

def getch():
    if select([sys.stdin],[],[],0) == ([sys.stdin],[],[]):
        return sys.stdin.read(1)
    return False



# Llamada a la funcion main
if __name__ == '__main__':

	global ser
	# Open the Serial Port.
	ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)

	envia(ser,'TestMode On', 0.2)

	envia(ser,'PlaySound 1', 0.3)

	envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)

	envia(ser,'SetLDSRotation On',0.1)

	old_settings = termios.tcgetattr(sys.stdin)
	tty.setcbreak(sys.stdin.fileno())

	# Parametros Robot.
	S = 121.5		# en mm
	distancia_L = 0	# en mm
	distancia_R = 0	# en mm
	speed = 300		# en mm/s
	tita_dot = 0
	tiempo = 20		
	direccion = 0

	print "########################"
	print "Speed = " + str(speed)
	print "Tita_dot = " + str(tita_dot)

	if direccion == 0:
		print "Direction: fordward."
	else:
		print "Direction: backward."

	print "q to exit."
	print "########################"

	# Tecla a leer.
	tecla = ''
	comando = ''

	test=False
	avoid=True
	detection_dist=2000
	
	while tecla != "q":

		# Leemos la tecla.
		# print "Write command: "       
		tecla = getch()
		if tecla == 'n':
			test=False
			print('test off')
		msg = envia(ser,'GetLDSScan',0.1)
		laser_values=[]
		for line in msg.split('\r\n')[2:362]:
			s = line.split(',')
			if int(s[3])==0 and int(s[1])<detection_dist:
				lr = [int(s[0]), int(s[1])]
				laser_values.append(lr)

		# print(laser_values)
		min_dist=detection_dist
		ang=(-1)
		for r in laser_values:
			if r[0]>180:
				dist=r[1]+180-(r[0]-180)
			else:
				dist=r[1]+r[0]
			if min_dist>dist:
				min_dist=dist
				ang=r[0]
		if ang==-1:
			ang=180
		theta=(math.radians(ang))-math.pi
		if theta < -3: theta = theta + 2*math.pi
		distancia_R = ((speed*0.8 + (S * theta)) * tiempo)
		distancia_L = ((speed*0.8 + (-S * theta)) * tiempo)
		if (ang>330 or ang<30) and min_dist<700:
			distancia_L=-6000
			distancia_R=-6000

		print "mindist = "+str(min_dist)
		print "ang = "+str(ang)
		# print "theta " + str(theta)
		print(distancia_L)
		print(distancia_R)
		comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed)
		envia(ser,comando, 0.2)
	direccion = 0
	speed = 0
	tita_dot = 0
	distancia_L = 0
	distancia_R = 0

	envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.2)
	envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
	envia(ser,'SetLDSRotation Off',0.1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)