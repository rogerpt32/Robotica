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
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:

        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:

        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	print ch
    return ch



# Llamada a la funcion main
if __name__ == '__main__':

	global ser
	# Open the Serial Port.
	ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)

	envia(ser,'TestMode On', 0.2)

	envia(ser,'PlaySound 1', 0.3)

	envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)

	envia(ser,'SetLDSRotation On',0.1)

	# Parametros Robot.
	S = 121.5		# en mm
	distancia_L = 0	# en mm
	distancia_R = 0	# en mm
	speed = 0 		# en mm/s
	tita_dot = 0
	tiempo = 20		
	direccion = 0

	x=0
	y=0
	d=0
	angle=0

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
	detection_dist=900
	fov=35
	
	while tecla != "q":

		# Leemos la tecla.
		print "Write command: "       
		tecla = getch()

		if tecla == "t":
			test=not test
			diff_R=0
			print('test mode')

		while test:
			msg = envia(ser,'GetLDSScan',0.1)
			laser_values=[]
			for line in msg.split('\r\n')[2:362]:
				s = line.split(',')
				if int(s[3])==0 and int(s[1])<detection_dist and (int(s[0])>(360-fov) or int(s[0])<fov):
					lr = [int(s[0]), int(s[1])]
					laser_values.append(lr)
			# print(laser_values)
			min_dist=detection_dist
			ang=(-1)
			for r in laser_values:
				if min_dist>r[1]:
					min_dist=r[1]
					ang=r[0]
			distancia_R = (speed * pow(-1, direccion))*tiempo
			distancia_L = (speed * pow(-1, direccion))*tiempo
			if ang>(360-fov):
				distancia_L=distancia_L*(float(min_dist)/float(detection_dist))
				diff_R=distancia_R-distancia_L
			elif ang!=-1:
				distancia_R=distancia_R*(float(min_dist)/float(detection_dist))
				diff_R=distancia_R-distancia_L
			else:
				if diff_R<0:
					distancia_L=distancia_L-min(speed*tiempo,-diff_R)
					diff_R=diff_R+min(speed*tiempo,-diff_R)
				else:
					distancia_R=distancia_R-min(speed*tiempo,diff_R)
					diff_R=diff_R-min(speed*tiempo,diff_R)
			print(distancia_L)
			print(distancia_R)
			comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
			envia(ser,comando, 0.2)

		if tecla == 'w' or tecla == 's':

			if tecla == 'w':
				speed = speed + 50
			else:
				speed = speed - 50

			if speed >= 0:
				direccion = 0
			else:
				direccion = 1

			if speed == 0:

				envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.2)
				envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)

			else:
				distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
				distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)

				comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
				envia(ser,comando, 0.2)

		elif tecla == 'a' or tecla == 'd':

			if tecla == 'a':
				tita_dot = tita_dot + (3.1415/10)
			else:
				tita_dot = tita_dot - (3.1415/10)

			distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
			distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)

			comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
			envia(ser,comando, 0.2)

		elif tecla == 'x':

			direccion = 0
			speed = 0
			tita_dot = 0
			distancia_L = 0
			distancia_R = 0

			envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.2)
			envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)


		if tecla == 'w' or tecla == 's' or tecla == 'd' or tecla == 'a' or tecla == 'x':
			
			#print "\n########################"
			#print 'Comando enviado: ' + comando
			print "########################"
			print "########################"
			print "Speed = " + str(speed)
			print "Tita_dot = " + str(tita_dot)
			print "x = "+ str(x) + ", y = "+ str(y)

			if direccion == 0:
				print "Direction: fordward."
			else:
				print "Direction: backward."
			print "########################"
	envia(ser,'SetLDSRotation Off',0.1)