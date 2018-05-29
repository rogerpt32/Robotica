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

def get_motors():
    """ Ask to the robot for the current state of the motors. """
    msg = envia(ser, 'GetMotors LeftWheel RightWheel').split('\n')
    
    # For better understanding see the neato commands PDF.
    
    L = int(msg[4].split(',')[1])
    R = int(msg[8].split(',')[1])
    
    return (L, R)

def odometry(L, R,x_world, y_world, theta_world):
	S = 121.5
	new_L, new_R = L - L_ini, R - R_ini
	#if new_R!=0:
	#	print(new_L, new_R, new_L/new_R)
	d = (new_R+new_L)/2
	theta = (new_R-new_L)/(2*S)
	theta_world = (theta_world+theta)%(2*math.pi)
	x_world = x_world+d*math.cos(theta_world)
	y_world = y_world+d*math.sin(theta_world)
	print(x_world,y_world,theta_world)
	return x_world, y_world, theta_world


def goto(y_neato,theta_neato): 
	# x and y where we want to go
	# x_neato and y_neato where we are
	speed=50
	S = 121.5
	tiempo = 5
	estat='RECUPERANT'

	if y_neato>100 or y_neato<-100:
		theta=math.atan2(float(-y_neato),500)
		theta=((theta-theta_neato)%(2*math.pi))
		if theta > math.pi:
			theta=theta-2*math.pi

		distancia_R = ((speed + (S * theta)) * tiempo)
		distancia_L = ((speed + (-S * theta)) * tiempo)
	elif theta_neato>0.1 or theta_neato<-0.1:
		print theta_neato
		theta=theta_neato
		if theta > math.pi:
			theta=theta-2*math.pi
		theta=-theta
		distancia_R = (S * theta)
		distancia_L = (-S * theta)
	else:
		estat='RECTE'
		distancia_R=(speed*tiempo)
		distancia_L=(speed*tiempo)
	comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed)
	envia(ser,comando)
	return estat



# Llamada a la funcion main
if __name__ == '__main__':

	global ser, estat, distancia_R, distancia_L, L_ini, R_ini
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
	L_ini,R_ini=get_motors()

	estat='RECTE'
	x_world=0
	y_world=0
	theta_world=0

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
	detection_dist=800
	fov=35
	
	while tecla != "q":

		# Leemos la tecla.
		# print "Write command: "       
		tecla = getch()

		if tecla == "t":
			test=True
			diff_R=0
			print('test mode')

		while test:
			L,R=get_motors()
			x_world,y_world,theta_world=odometry(L,R,x_world,y_world,theta_world)
			L_ini=L
			R_ini=R
			tecla = getch()
			if tecla == 'n':
				test=False
				print('test off')
			msg = envia(ser,'GetLDSScan',0.1)
			laser_values=[]
			wall=[]
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
			distancia_R = speed*tiempo
			distancia_L = speed*tiempo
			if avoid==True:
				if ang>(360-fov):
					distancia_L=distancia_L*(float(min_dist-250)/float(detection_dist))
					if estat!='RECUPERANT':
						estat='RECUPERANT'
				elif ang!=-1:
					distancia_R=distancia_R*(float(min_dist-250)/float(detection_dist))
					if estat!='RECUPERANT':
						estat='RECUPERANT'
				elif estat=='RECUPERANT':
						estat = goto(y_world,theta_world)
						continue
			comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed)
			envia(ser,comando, 0.5)
			print estat

		if tecla == 'w':
			
			speed = speed + 50

			if speed == 0:

				envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.2)
				envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)

			else:
				distancia_R = (speed + (S * tita_dot)) * tiempo
				distancia_L = (speed + (-S * tita_dot)) * tiempo

				comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed)
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
			print "Speed = " + str(speed)
			print "Tita_dot = " + str(tita_dot)
			print "########################"
	envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.2)
	envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
	envia(ser,'SetLDSRotation Off',0.1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)