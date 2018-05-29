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
	avoid=False
	follow=True
	detection_dist=1000
	fov=35
	fov_block=20
	
	while tecla != "q":

		# Leemos la tecla.
		# print "Write command: "       
		tecla = getch()

		if tecla == "t":
			test=True
			diff_R=0
			print('test mode')

		while test:
			print("Entering while again")
			t=time.time()
			tecla = getch()
			print("Past tecla = getch()"+str(time.time()-t))
			if tecla == 'n':
				test=False
				print('test off')
			msg = envia(ser,'GetLDSScan',0.1)
			laser_values=[]
			wall_front=[]
			wall_left=[]
			wall_fl=[]
			for line in msg.split('\r\n')[2:362]:
				s = line.split(',')
				if int(s[3])==0 and int(s[1])<detection_dist and (int(s[0])>(360-fov) or int(s[0])<fov):
					lr = [int(s[0]), int(s[1])]
					laser_values.append(lr)
				
				if int(s[3])==0 and int(s[1])<detection_dist and int(s[0])>(90-fov_block) and int(s[0])<(90+fov_block):
					lr = [int(s[0]), int(s[1])]
					wall_left.append(lr)
				if int(s[3])==0 and int(s[1])<detection_dist and (int(s[0])>(360-fov_block) or int(s[0])<fov_block):
					lr = [int(s[0]), int(s[1])]
					wall_front.append(lr)
				if int(s[3])==0 and int(s[1])<detection_dist and int(s[0])>fov_block and int(s[0])<(90-fov_block):
					lr = [int(s[0]), int(s[1])]
					wall_fl.append(lr)

			# print(laser_values)
			distancia_R = (speed * pow(-1, direccion))*tiempo
			distancia_L = (speed * pow(-1, direccion))*tiempo
			if avoid==True:
				min_dist=detection_dist
				ang=(-1)
				for r in laser_values:
					if min_dist>r[1]:
						min_dist=r[1]
						ang=r[0]
				if ang>(360-fov):
					distancia_L=distancia_L*(float(min_dist-250)/float(detection_dist))
					diff_R=diff_R+distancia_R-distancia_L
				elif ang!=-1:
					distancia_R=distancia_R*(float(min_dist-250)/float(detection_dist))
					diff_R=diff_R+distancia_R-distancia_L
				else:
					if diff_R<0:
						distancia_L=distancia_L-min(speed*tiempo,-diff_R)
						diff_R=diff_R+min(speed*tiempo,-diff_R)
					else:
						distancia_R=distancia_R-min(speed*tiempo,diff_R)
						diff_R=diff_R-min(speed*tiempo,diff_R)
			elif follow==True:
				f_dist=detection_dist
				for r in wall_front:
					if f_dist>r[1]:
						f_dist=r[1]
				print 'f = ' + str(f_dist)
				if f_dist<600:
					distancia_R=-distancia_R
				else:
					fl_dist=detection_dist
					for r in wall_fl:
						if fl_dist>r[1]:
							fl_dist=r[1]
					print 'fl = ' + str(fl_dist)
					if fl_dist<550:
						distancia_R=distancia_R*((float(fl_dist)-250)/300.0)
					else:
						l_dist=detection_dist
						for r in wall_left:
							if l_dist>r[1]:
								l_dist=r[1]
						print 'l = ' + str(l_dist)
						if not fl_dist < l_dist:
							if l_dist<300:
								distancia_R=distancia_R*(float(l_dist)-150/150.0)
							elif l_dist>450:
								distancia_L=distancia_L*max(0.5,300.0/float(l_dist))
			#print(distancia_L)
			#print(distancia_R)
			t=time.time()
			comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
			envia(ser,comando, 0.1)
			print("set motor"+str(time.time()-t))

		if tecla == 'w' or tecla == 's':

			if tecla == 'w':
				speed = speed + 50
			else:
				speed = speed - 30

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
				envia(ser,comando, 0.1)

		elif tecla == 'a' or tecla == 'd':

			if tecla == 'a':
				tita_dot = tita_dot + (3.1415/10)
			else:
				tita_dot = tita_dot - (3.1415/10)

			distancia_R = (((speed * pow(-1, direccion) ) + (S * tita_dot)) * tiempo) * pow(-1, direccion)
			distancia_L = (((speed * pow(-1, direccion) ) + (-S * tita_dot)) * tiempo) * pow(-1, direccion)

			comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed * pow(-1, direccion))
			envia(ser,comando, 0.1)

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
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)