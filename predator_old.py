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

def dist(a,b): #cyclic distance for angles
    i = (a - b) % 360
    j = (b - a) % 360
    return min(i, j)

def similar(a,b):
	return abs(a-b)<200
	



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
			if min_dist>r[1]:
				near=0
				for aprox in laser_values:
					if dist(aprox[0],r[0])<10 and similar(aprox[1],r[1]):
						near+=1
				if near<10:
					min_dist=r[1]
					ang=r[0]
		if ang==-1:
			ang=0
		theta=math.radians(ang) if ang <180 else -math.radians(ang-180)
		distancia_R = ((speed + (S * theta)) * tiempo)
		distancia_L = ((speed + (-S * theta)) * tiempo)
		#if ang>325 or ang<35:
		#	distancia_R=-distancia_L
		#	distancia_L=-distancia_R

		print "theta " + str(theta)
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