#!/usr/bin/env python
# Autor> Ruben Claveria Vega
# Curso> EL5206 Laboratorio de Inteligencia Computacional y Robotica

# En lugar de 'testpy', deberia ir el nombre del paquete creado.
import roslib; roslib.load_manifest('EL5206')
import robot_utilities
import time
import rospy
import math
import numpy
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from std_msgs.msg import String 

class Controller:
	def __init__(self):

		# inicializa nodo ROS
		rospy.init_node('Simulation');

		# robotID = ID del robot que se desea controlar
		# si hay solo un robot, robotID debe ser ""
		# si hay mas de un robot, robotID debe ser de la forma "/robot_0", "/robot_1", etc.
		robotID = ""

		# Posicion y orientacion iniciales.
		# IMPORTANTE: deben ser consistentes con la del archivo .world
		init_x = -2.0
		init_y = -2.0
		init_angle = 90.0

		# creacion de un objeto de tipo Robot
		R = robot_utilities.Robot(robotID, init_x, init_y, init_angle)

		# Subscripcion a los topicos de interes: odometria (Odometry) y sensor de distancia (LaserScan)
		rospy.Subscriber(robotID+'/odom', Odometry, R.odom_callback)
		rospy.Subscriber(robotID+'/base_scan', LaserScan, R.ranger_callback)
		# Observacion: Las funciones de callback se ejecutan cada vez que el programa recibe informacion 
		# desde un topico. Se utilizan, generalmente, para actualizar datos.
		# Pueden utilizarse funciones de callback programadas por el alumno, 
		# diferentes a las provistas por la clase Robot

		# Se recomienda que este ciclo vaya antes de todas las instrucciones de control
		# Sirve para asegurar que el Robot recibio su primera lectura de sensor
		while not rospy.is_shutdown() and len(R.distances)==0:
			R.rate.sleep()

		# Ciclo de ejemplo. 
		# Si se va a controlar el robot con una funcion de robot_utilities en vez de con el ciclo:
		# opcion 1: cambiar True por False en la condicion del while
		# opcion 2: comentar el bloque de codigo
		while not rospy.is_shutdown() and False:
			x=R.pose_x
			y=R.pose_y
			theta=R.angle
			# Define velocidad lineal [metros/segundo] del robot durante la iteracion
			#R.cmd.linear.x = 0.2
			R.nSteps(0,1500)

			# Velocidad angular [radianes/segundo]; rango aceptable: [-pi/2, pi/2 ], aprox. [-1.57, 1.57]
			R.cmd.angular.z = 0.0 

			# indica a Stage la velocidad que el robot tendra durante un ciclo
			R.cmd_pub.publish(R.cmd)
			
			# Ejemplo de como rescatar datos captados por los sensores
			# En este caso solo se imprimen, pero la idea es utilizarlos en decisiones de control
			print "_______________________"
			#print 'Primera lectura del laser [m]: '+ str(R.distances[0])
			#print 'Posicion robot [m]: x = ' + str(R.pose_x)+'; y = ' +str(R.pose_y)
			print 'Delta theta: ' + str(R.angle-theta)
			print 'Delta X: ' + str(R.pose_x-x)
			print 'Delta Y: ' + str(R.pose_y-y)
			#R.show_distance()
			#print str(R.angle)
			# Mantiene la regularidad temporal (de acuerdo al rate definido en robot_utilities.Robot)
			R.rate.sleep()
		

		##PARTE 2	
		#print 'Posicion inicial : x = ' + str(R.pose_x)+'; y = ' +str(R.pose_y) #'; t = ' + str(R.angle)
		#xi=R.pose_x ; yi=R.pose_y
		#ti=R.angle
		#print str(R.angle)
		#R.nSteps(900,200)
		#print 'Posicion robot [m]: x = ' + str(R.pose_x)+'; y = ' +str(R.pose_y) #'; t = ' +str(R.angle)
		#print str(R.angle)
		#xf=R.pose_x ; yf=R.pose_y
		#tf=R.angle
		#print 'Delta x = ' + str(xf-xi)
		#print 'Delta y = ' + str(yf-yi)
		#print 'Delta t = ' + str(tf-ti)



		# Funciones de ejemplo (uncomment para usar): 
		#R.nSteps(1500,1500)
		#R.nSteps(1500,900)
		#R.nSteps(900,1500)
		#R.moveTill(math.pi*5.0/2, 0.5, 0.1)

		

		##DATOS PARA PARTE 3 y 4

		xa=-2.0
		ya=-2.0
		ta=90.0
		xb=4.0
		yb=1.0
		w=0.404
		rel=1.818*10**(-4)  ##r/N
		
		##PARTE 3
		
		#print 'Posicion robot inicial [m]: x = ' + str(R.pose_x)+'; y = ' +str(R.pose_y) 
	
		#tb=math.atan((yb-ya)/(xb-xa))
		#deltaA=math.degrees(tb)-ta

		#Primer Movimiento

		#n1=(math.radians(deltaA)*w)/(4*math.pi*rel)
		#R.nSteps(-n1,n1)
		

		#Segundo Movimiento
		
		#d=math.sqrt((xa-xb)**2+(yb-ya)**2)
		#n2=d/(2*math.pi*rel)
		#R.nSteps(n2,n2)	

		#print 'Posicion robot final [m]: x = ' + str(R.pose_x)+'; y = ' +str(R.pose_y) 
		#R.show_distance()
        

		##PARTE 4

		
		deltax=xb-xa
		deltay=yb-ya
		if deltax<0:
			tb=math.pi+atan(deltay/deltax)
		else:
			tb=math.atan(deltay/deltax)
		
		d= math.sqrt(deltax**2+deltay**2)

		alpha=2*(tb-ta)

		R=d/(2*math.sin(abs(alpha/2)))
		

		nl=(alpha*(2*R-w))/(4*math.pi*rel)
		nr=(alpha*(2*R+w))/(4*math.pi*rel)
		
		#print 'Posicion robot inicial [m]: x = ' + str(R.pose_x)+'; y = ' +str(R.pose_y)
		
		R.nSteps(int(round(nl)),int(round(nr)))
	
		print 'Posicion robot final [m]: x = ' + str(R.pose_x)+'; y = ' +str(R.pose_y) 
		R.show_distance()

if __name__ == "__main__": Controller()
