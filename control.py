#Librerias necesarias para el desarrollo del proyecto
import rospy
from heron_msgs.msg import Drive
import curses
import math
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import csv
import subprocess


def letra_numero(var):
    """
    Esta función se emplea para mapear ciertos caracteres del teclado 
    a valores numéricos específicos, que corresponden con las posibles posiciones
    en el mapa del Heron.
        z --> -10
        x --> -5
        c --> 0
        v --> 5
        b --> 10
        
    """
    if var == ord('z'):
        posicion = -10
    if var == ord('x'):
        posicion = -5
    if var == ord('c'):
        posicion = 0
    if var == ord('v'):
        posicion = 5
    if var == ord('b'):
        posicion = 10
                

    return posicion    


def publish_to_topic(stdscr):

    # Inicializar el nodo
    rospy.init_node('cmd_drive', anonymous=True)

    # Crear un publicador para el topic 'cmd_drive' con mensajes de tipo Drive
    pub = rospy.Publisher('/cmd_drive', Drive, queue_size=10)

    #Configurcaion cursed
    stdscr.clear()
    stdscr.addstr("Presiona las teclas para controlar flechas")

    #Indica la frecuencia de actualizacion
    rate = rospy.Rate(10) 
    drive_msg = Drive()

    #Inicializar las variables
    velocidad_izq = 0
    velocidad_der = 0
    vel_preventiva=1


    while True:
        """
        En este archivo se realiza una navegación manual y automática del Heron 
        a traves de los comandos de teclado:
             q --> salir del bucle
             s --> parar
             d --> modo automático
             ⭡ --> avanza
             ⭣ --> retrocede
             ⭠ --> gira a la izquierda
             ⭢ --> gira a la derecha
        """
        key = stdscr.getch()

        if key == ord('q'): #Sale del bucle
            break

        if key == ord('s'): #Parada
            drive_msg.left = 0.0
            drive_msg.right = 0.0  

        if key == ord('d'): #Modo de funcionamiento autónomo en el cual el usuario le indicará las coordenadas a la que desea que Heron se mueva
            stdscr.addch('\n')
            stdscr.addstr(" z --> -10 \n x --> -5 \n c --> 0 \n v --> 5 \n b --> 10 \n")
            stdscr.addstr("Ingrese la coord. X deseada: ")
            var1 = stdscr.getch()
            var1 = letra_numero(var1) 
            stdscr.addstr(str(var1))

            stdscr.addch('\n')
            stdscr.addstr("Ingrese la coord. Y deseada: ")
            var2 = stdscr.getch()
            var2 = letra_numero(var2)
            stdscr.addstr(str(var2))
            stdscr.getch()
            #stdscr.addch('\n')
            #print(var2)
        
            if var1 == 0 and var2 < 0: # No son accesibles porque se han colocado obstáculos en esas coordenadas             
                stdscr.addstr("Coordenada no accesible")
            else: #Se mandan las coordenadas recibidas al archivo que genera las trayectorias y el control del barco
                subprocess.call(['python', 'p2p_interp_suavizado_astar.py', str(var1), str(var2)])

        if key == curses.KEY_UP: #Avanzar
            velocidad_der = max(0,min(velocidad_der,velocidad_izq,vel_preventiva))
            velocidad_izq = max(0,min(velocidad_der,velocidad_izq,vel_preventiva))

            velocidad_izq += 0.2
            velocidad_der += 0.2

            drive_msg.left = velocidad_izq
            drive_msg.right = velocidad_der

        elif key == curses.KEY_DOWN: #Retroceder
            velocidad_der = min(velocidad_der,velocidad_izq,vel_preventiva)
            velocidad_izq = min(velocidad_der,velocidad_izq,vel_preventiva)

            velocidad_izq -= 0.2
            velocidad_der -= 0.2

            drive_msg.left = velocidad_izq
            drive_msg.right = velocidad_der

        elif key == curses.KEY_LEFT: #Girar a la izquierda
            velocidad_izq = velocidad_izq - 0.1
            velocidad_der = velocidad_der + 0.1

            drive_msg.left = velocidad_izq
            drive_msg.right = velocidad_der

        elif key == curses.KEY_RIGHT: #Girar a la derecha
            velocidad_izq = velocidad_izq + 0.1
            velocidad_der = velocidad_der - 0.1

            drive_msg.left = velocidad_izq
            drive_msg.right = velocidad_der

        # Publicar el mensaje en el topic
        pub.publish(drive_msg)

        #stdscr.clear()
        stdscr.refresh()

        rate.sleep()



if __name__ == '__main__':

    try:
        curses.wrapper(publish_to_topic)
    except rospy.ROSInterruptException:
        pass

