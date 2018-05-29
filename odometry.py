#!/usr/bin/python
#coding: utf8

"""
Imports
"""
import time
import math
import serial
import json
import atexit
import http_viewer

"""
Imports de Teclado
"""
import os
import sys, tty, termios
from select import select
from multiprocessing import Process, Queue

# To import the function "envia" from the file "test_NeatoCommands.py"
from test_NeatoCommands import envia

def all_stop(viewer):
	envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.2)
	envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)
	envia(ser,'SetLDSRotation Off',0.1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
	envia(ser, 'TestMode Off', 0.2)
	ser.close()
	viewer.quit()

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

def get_laser():
	msg = envia(ser,'GetLDSScan',0.1)
	laser_values=[]
	for line in msg.split('\r\n')[2:362]:
		s = line.split(',')
		if int(s[3])==0:
			lr = [int(s[0]), int(s[1])]
			laser_values.append(lr)
	return laser_values

def parse_laser(xw,yw,tw,lv):
	lxy=[]
	for [a,d] in lv:
		x=xw+d*math.cos(tw+math.radians(a))
		y=yw+d*math.sin(tw+math.radians(a))

		x=x/20
		y=y/20
		lxy.append([x,y])
	try:
		with open("laser.json","r") as f:
			j=json.loads(f.read())
			data=j["laserPoints"]
	except:
		data=[]
	for row in lxy:
		data.append({'x':row[0],'y':row[1]})
	ljson=open("laser.json","w")
	ljson.write(json.dumps({"laserPoints": data},indent=4,sort_keys=True))
	ljson.close()

def save_pose(xw,yw):
	try:
		with open("pose.json","r") as f:
			j=json.loads(f.read())
			data=j["pose"]
	except:
		data=[]
	data.append({'x':xw/20.0,'y':yw/20.0})
	ljson=open("pose.json","w")
	ljson.write(json.dumps({"pose": data},indent=4,sort_keys=True))
	ljson.close()

def goto(x,y,x_neato,y_neato,theta_neato,safe_p): 
	# x and y where we want to go
	# x_neato and y_neato where we are
	speed=100
	S = 121.5
	tiempo = 5

	safe=400
	error_margin=20
	x=x+safe

	if (((x-x_neato)>error_margin*2 or (x-x_neato)<-error_margin*2) or ((y-y_neato)>error_margin or (y-y_neato<-error_margin))) and not safe_p:
	# d=math.sqrt(math.pow((x-x_neato),2)+math.pow((y-y_neato),2))
		if (((x-x_neato)>error_margin*10 or (x-x_neato)<-error_margin*10) or ((y-y_neato)>error_margin*10 or (y-y_neato<-error_margin*10))):
			speed=200
		theta=math.atan2(float(y-y_neato),float(x-x_neato))
		print "x = "+str(x-x_neato)+" y = "+ str(y-y_neato)
		# print "theta_neato " + str(theta_neato)
		theta=((theta-theta_neato)%(2*math.pi))
		if theta > math.pi:
			theta=theta-2*math.pi
		print "theta " + str(theta)

		distancia_R = ((speed + (S * theta)) * tiempo)
		distancia_L = ((speed + (-S * theta)) * tiempo)
	elif theta_neato>0.1 or theta_neato<-0.1:
		safe_p=True
		theta=theta_neato
		if theta > math.pi:
			theta=theta-2*math.pi
		theta=-theta
		distancia_R = (S * theta)
		distancia_L = (-S * theta)
	else:
		lv=get_laser()
		min_dist=safe*2
		for l in lv:
			if l[0]>177 and l[0]<183 and l[1]<min_dist:
				min_dist=l[1]
		if min_dist<safe*2:
			min_dist=min_dist-50
			distancia_L=-min_dist
			distancia_R=-min_dist
			comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed)
			envia(ser,comando)
			time.sleep(tiempo*2)
		return safe_p

	comando = 'SetMotor LWheelDist ' + str(distancia_L) + ' RWheelDist ' + str(distancia_R) + ' Speed ' + str(speed)
	envia(ser,comando)
	return safe_p




# Llamada a la funcion main
if __name__ == '__main__':
	try:
		global ser, L_ini, R_ini, safe_p
		# Open the Serial Port.
		ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)

		envia(ser,'TestMode On', 0.2)

		envia(ser,'PlaySound 1', 0.3)

		envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)

		envia(ser,'SetLDSRotation On',0.1)

		old_settings = termios.tcgetattr(sys.stdin)
		tty.setcbreak(sys.stdin.fileno())

		port_web_server=123
		r_queue = Queue()
		l_queue = Queue()
		viewer = http_viewer.HttpViewer(port_web_server, l_queue, r_queue)
		#viewer.main_http_server()

		# Parametros Robot.
		S = 121.5		# en mm
		distancia_L = 0	# en mm
		distancia_R = 0	# en mm
		speed = 0 		# en mm/s
		tita_dot = 0
		tiempo = 20		
		direccion = 0

		L_ini, R_ini = get_motors()
		x_world=0
		y_world=0
		theta_world=0
		safe_p=False

		if os.path.isfile("./laser.json"):
			os.remove("./laser.json")
		if os.path.isfile("./pose.json"):
			os.remove("./pose.json")
		print ""
		print "########################"
		print "q to exit."
		print "########################"

		# Tecla a leer.
		tecla = ''
		comando = ''
		go=False

		while tecla != "q":

			# Leemos la tecla.
			# print "Write command: "       
			tecla = getch()
			L, R = get_motors()
			(x_world,y_world,theta_world)=odometry(L, R, x_world, y_world, theta_world)
			save_pose(x_world,y_world)
			L_ini=L
			R_ini=R
			if not go:
				lv=get_laser()
				min_dist=600
				for l in lv:
					if l[0]>177 and l[0]<183 and l[1]<min_dist:
						min_dist=l[1]
						print l[1]
				print min_dist
				parse_laser(x_world,y_world,theta_world,lv)

			if go:
				safe_p=goto(0,0,x_world,y_world,theta_world,safe_p)

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
				print "Speed = " + str(speed)
				print "Tita_dot = " + str(tita_dot)

				if direccion == 0:
					print "Direction: fordward."
				else:
					print "Direction: backward."
				print "########################"
			if tecla == 'g':
				direccion = 0
				speed = 0
				tita_dot = 0
				distancia_L = 0
				distancia_R = 0

				envia(ser,'SetMotor LWheelDisable RWheelDisable', 0.2)
				envia(ser,'SetMotor RWheelEnable LWheelEnable', 0.2)

				go=True
			if tecla == 'c':
				go=False

		atexit.register(all_stop,viewer)
	except KeyboardInterrupt:
		all_stop(viewer)