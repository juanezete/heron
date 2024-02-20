import rospy
from heron_msgs.msg import Drive
import curses

#!/usr/bin/env python


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

        key = stdscr.getch()

        if key == ord('q'):
            break

        if key == ord('s'):
            drive_msg.left = 0.0
            drive_msg.right = 0.0  

        if key == curses.KEY_UP:
            velocidad_der = max(0,min(velocidad_der,velocidad_izq,vel_preventiva))
            velocidad_izq = max(0,min(velocidad_der,velocidad_izq,vel_preventiva))

            velocidad_izq += 0.2
            velocidad_der += 0.2

            drive_msg.left = velocidad_izq
            drive_msg.right = velocidad_der

        elif key == curses.KEY_DOWN:
            velocidad_der = min(velocidad_der,velocidad_izq,vel_preventiva)
            velocidad_izq = min(velocidad_der,velocidad_izq,vel_preventiva)

            velocidad_izq -= 0.2
            velocidad_der -= 0.2

            drive_msg.left = velocidad_izq
            drive_msg.right = velocidad_der

        elif key == curses.KEY_LEFT:
            velocidad_izq = velocidad_izq - 0.1
            velocidad_der = velocidad_der + 0.1

            drive_msg.left = velocidad_izq
            drive_msg.right = velocidad_der

        elif key == curses.KEY_RIGHT:
            velocidad_izq = velocidad_izq + 0.1
            velocidad_der = velocidad_der - 0.1

            drive_msg.left = velocidad_izq
            drive_msg.right = velocidad_der

        # Publicar el mensaje en el topic
        pub.publish(drive_msg)

        stdscr.clear()
        stdscr.refresh()

        rate.sleep()



if __name__ == '__main__':

    try:
        curses.wrapper(publish_to_topic)
    except rospy.ROSInterruptException:
        pass
