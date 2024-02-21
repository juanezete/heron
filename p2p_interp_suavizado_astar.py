import rospy
import math
from sensor_msgs.msg import Imu
from heron_msgs.msg import Drive
from nav_msgs.msg import Odometry
import csv
import sys
from scipy.signal import savgol_filter
import numpy as np
import heapq
from calculapuntos import cargar_obstaculos, obtener_puntos_paso



def exportar_a_csv(filename, data):
        """"
        Recopilacion de datos para generar graficas
        """
        with open(filename, 'w') as csvfile:
            fieldnames = ['Tiempo', 'Posicion X', 'Posicion Y', 'Objetivo X', 'Objetivo Y', 'Orientacion', 'Angulo objetivo']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
            writer.writeheader()
            for row in data:
                writer.writerow({
                    'Tiempo': row[0],
                    'Posicion X': row[1],
                    'Posicion Y': row[2],
                    'Objetivo X': row[3],
                    'Objetivo Y': row[4],
                    'Orientacion': row[5],
                    'Angulo objetivo': row[6],

                })

historial = []

def cargar_matriz_desde_txt(nombre_archivo):
    """
    Se encarga de abrir archivos txt y almanacenarlos en una variable

    """
    matriz_costos = []

    with open(nombre_archivo, 'r') as archivo:
        for linea in archivo:
            # Eliminamos corchetes y dividimos por comas
            valores = linea.replace('[', '').replace(']', '').split(',')
            
            # Eliminamos espacios en blanco y valores vacios
            valores = [valor.strip() for valor in valores if valor.strip()]

            # Imprimimos los valores antes de convertirlos
            #print("Valores antes de convertir:", valores)
            
            # Convertimos los valores a numeros flotantes
            fila = [float(valor) for valor in valores]
            
            matriz_costos.append(fila)

    return matriz_costos

prev_error = 0
integral = 0

# Variables globales para almacenar la posicion y orientacion
pos_x = 0
pos_y = 0
ori_z = 0

def gps(data): 
    """"
    Esta funcion se encarga de obtener la posicion x, y a traves del topic data/position
    """
    global pos_x
    global pos_y
    global tiempo_sim

    # Utilizar la posicin de Odometry
    pos_x = data.pose.pose.position.x
    pos_y = data.pose.pose.position.y

    tiempo_sim = rospy.Time.now()

def imu_callback(data):
    """"
    Esta funcion se encarga de obtener la orientacion y pasarla de cuateriones
    a angulos de euler a traves del topic data/orientation
    """
    global ori_z
    # Utilizar la orientacin de la IMU
    _,_,ori_z = quaternion_a_angulos_de_euler(data.orientation)
   
    publicar()

def quaternion_a_angulos_de_euler(quaternion):
    """"
    Esta funcion se encarga de convertir cuaterion a angulos de Euler
    """
    
    t0 = +2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z)
    t1 = +1.0 - 2.0 * (quaternion.x**2 + quaternion.y**2)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    t4 = +1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def control_pid(distancia, kp, ki, kd):
    """"
    Esta funcion realiza un control PID a través de la distancia para
    obtener la velocidad de giro
    """
    global prev_error
    global integral

    #error = ang_obj - ori_z
    error = distancia
    integral += error
    derivative = error - prev_error

    # Parametros del PID
    velocidad_giro = kp * error + ki * integral + kd * derivative

    prev_error = error

    return velocidad_giro


def publicar():
    """"
    Esta funcion se encarga de publicar en topic cmd/drive los valores de la 
    velocidad de los motores derecha e izquierda
    
    """
    global x_fin 
    global y_fin 
    global ang_obj
    global punto_actual_trayectoria 
    global trayectoria


    pub = rospy.Publisher('/cmd_drive', Drive, queue_size=1)

    # Indica la frecuencia de actualizacin
    drive_msg = Drive()

    # Punto objetivo final
    x_fin = trayectoria[punto_actual_trayectoria][0]
    y_fin = trayectoria[punto_actual_trayectoria][1]

    # Inicializar las variables de velocidad
    velocidad_izq = 0
    velocidad_der = 0

    # Ganancia proporcional angulo y distancia
    kpd = 0.1
    kpg = 0.1

    # Calculo distancia
    distancia = math.sqrt((x_fin - pos_x)**2 + (y_fin - pos_y)**2)
    # Calculo angulo
    ang_obj = math.atan2(y_fin - pos_y, x_fin - pos_x)
    error_ang = ang_obj - ori_z

    kp = 0.075        #0.05
    ki = 0.0    #0.001
    kd = 0.0
    
    giro_fuerte = control_pid(distancia, kp, ki, kd)
    #print(giro_fuerte, kp*distancia)
    giro_debil = 0
    if ori_z >= math.pi/2 and ori_z*ang_obj < 0:
        ori_z_desplazada = ori_z - math.pi
        ang_obj_desplazado = ang_obj - math.pi/2
        if ori_z_desplazada > ang_obj_desplazado + 0.1:
            velocidad_der += giro_fuerte
            velocidad_izq += giro_debil
        elif ori_z_desplazada < ang_obj_desplazado - 0.1:
            velocidad_der += giro_debil   
            velocidad_izq += giro_fuerte
    elif ori_z <= -math.pi/2 and ori_z*ang_obj < 0:
        ori_z_desplazada = ori_z - math.pi
        ang_obj_desplazado = ang_obj - math.pi/2
        if ori_z_desplazada > ang_obj_desplazado + 0.1:
            velocidad_der += giro_fuerte
            velocidad_izq += giro_debil
        elif ori_z_desplazada < ang_obj_desplazado - 0.1:
            velocidad_der += giro_debil   
            velocidad_izq += giro_fuerte
    else:
        if ori_z > ang_obj + 0.1:
            velocidad_der += giro_debil   
            velocidad_izq += giro_fuerte
        elif ori_z < ang_obj - 0.1:
            velocidad_der += giro_fuerte   
            velocidad_izq += giro_debil
        else: 
            velocidad_der = abs(distancia)*kpd
            velocidad_izq = abs(distancia)*kpd

    error_vel = kpd * distancia


#    print("distancia: ", distancia)
#    print("ori_z: ", ori_z)
#    print("ang_obj: ", ang_obj)
#    print("error angular: ", error_ang)
#    print("vel_izq: " , velocidad_izq)
#    print("vel_der: ", velocidad_der)
    

    # Publicar el mensaje en el topic
    drive_msg.left = velocidad_izq
    drive_msg.right = velocidad_der

    #Para indicar que ha llegado a su objetivo y parar los motores   
    if distancia < 1.5 and punto_actual_trayectoria == len(trayectoria) - 1:
        drive_msg.left = 0
        drive_msg.right = 0
        print("He terminado")
        rospy.signal_shutdown("Trayectoria completada")
    

    pub.publish(drive_msg)

    # Recopilamos los datos e importamos a csv
    historial.append([rospy.Time.now(), pos_x, pos_y, x_fin, y_fin, ori_z, ang_obj]) 
    exportar_a_csv('datos.csv', historial) 

    if distancia < 2 and punto_actual_trayectoria < len(trayectoria) - 1: #pasar las siguiente punto
        punto_actual_trayectoria += 1


def a_estrella(matriz_costos, puntos, inicio, objetivo, heuristica):
    """
    Genera la trayectoria dado la matriz de costes, los posibles puntos de acceso,
    el punto de inicio y el objetivo y la heuristica para evaluar la "calidad" de un 
    estado con respecto al estado objetivo

    """
    indices = {p: i for i, p in enumerate(puntos)}

    # Inicializar estructuras de datos
    n = len(puntos)
    visitado = [False] * n
    distancia = [float('inf')] * n
    padre = [-1] * n

    # Inicializar distancia al nodo inicial como 0
    distancia[indices[inicio]] = 0

    cola_prioridad = [(0, indices[inicio])]

    # Bucle principal
    while cola_prioridad:
        # Obtener el nodo con la menor distancia + heuristica
        _, u = heapq.heappop(cola_prioridad)
        if visitado[u]:
            continue

        visitado[u] = True

        # Verificar si hemos alcanzado el objetivo
        if puntos[u] == objetivo:
            # Reconstruir el camino y devolverlo
            camino = []
            while u != -1:
                camino.append(puntos[u])
                u = padre[u]
            return camino[::-1]

        # Actualizar las distancias de los nodos vecinos no visitados
        for v in range(n):
            if not visitado[v] and distancia[u] + matriz_costos[u][v] < distancia[v]:
                distancia[v] = distancia[u] + matriz_costos[u][v]
                padre[v] = u

                # Calcular el valor de f = g + h y agregar el nodo a la cola de prioridad
                f_valor = distancia[v] + heuristica(puntos[v], objetivo)
                heapq.heappush(cola_prioridad, (f_valor, v))

    # Si no se encuentra camino, devolver una lista vacia
    return []



def heuristica_euclidiana(punto_actual, objetivo):
    """
    Distancia euclidiana entre los puntos
    """
    return ((punto_actual[0] - objetivo[0]) ** 2 + (punto_actual[1] - objetivo[1]) ** 2) ** 0.5


def interpolacion_lineal_entre_puntos(punto1, punto2, cantidad_puntos):
    """
    Obtener nuevos puntos y que se encuentren comprendidos entre dos puntos dados 
    """
    puntos_interpolados = []
    factor_interpolacion = [0, 0.25, 0.5, 0.75]

    for i in range(1, cantidad_puntos + 1):
        #factor_interpolacion = i / (cantidad_puntos )
        x_interpolado = punto1[0] + factor_interpolacion[i] * (punto2[0] - punto1[0])
        y_interpolado = punto1[1] + factor_interpolacion[i] * (punto2[1] - punto1[1])
        puntos_interpolados.append((x_interpolado, y_interpolado))        #, punto2[3]))
    
    return puntos_interpolados

def interpolar_camino(path):
    """
    Esta funcion devuelve la nueva trayectoria a partir de los puntos dados incialmente
    """
    path_interpolado = [path[0]]
    
    for i in range(1, len(path)):
        punto_anterior = path[i - 1]
        punto_actual = path[i]
        
        # Interpolar entre puntos anteriores y actuales
        puntos_interpolados = interpolacion_lineal_entre_puntos(punto_anterior, punto_actual, cantidad_puntos=3)
        
        # Agregar puntos interpolados al nuevo camino
        path_interpolado.extend(puntos_interpolados)
        path_interpolado.append(punto_actual)
    
    return path_interpolado

def suavizar_trayectoria(trayectoria, ventana, orden):
    """
    Haciendo uso del filtro savgol podemos suavizar la trayectoria para que no haga giros bruscos 
    """
    x, y = zip(*trayectoria)
    x_suavizado = savgol_filter(x, ventana, orden)
    y_suavizado = savgol_filter(y, ventana, orden)
    trayectoria_suavizada = list(zip(x_suavizado, y_suavizado))
    return trayectoria_suavizada


def main():
    """
    Bucle fundamental del proyecto el usuario escogera la posicion objetivo,
    se escogerá la trayectoria mediante el método A* y mediante un control PID,
    la interpolación de los puntos y posteriomente suavizar la trayectoria, 
    se podrá observar cómo Heron se desplaza al punto objetivo.
    """
    global trayectoria
    global punto_actual_trayectoria 
    global pos_x
    global pos_y
    trayectoria = None 

    # Inicializar el nodo
    rospy.init_node('control', anonymous=True)

    # Suscribirse al topic de posicin de Odometry
    rospy.Subscriber("/odometry/filtered", Odometry, gps, queue_size=1)


    punto_actual_trayectoria = 0

    #Punto de inicio y objetivo
    while not rospy.is_shutdown() and pos_x == 0 and pos_y == 0:
        rospy.sleep(0.1)

    inicio = (round(pos_x * 0.2) * 5, round(pos_y * 0.2) * 5)
    print("\n")
    print('Inicio X: ', round(pos_x * 0.2) * 5)
    print('Inicio Y: ', round(pos_y * 0.2) * 5)

    #target_x = int(input("Ingrese la coordenada X deseada: "))
    #target_y = int(input("Ingrese la coordenada Y deseada: "))
    target_x = int(sys.argv[1])
    target_y = int(sys.argv[2])
    objetivo = (target_x, target_y)

    #Cargar el mapa de obstaculos y obtener la matriz de puntos posibles para desplazarse
    mapa_lado = 13
    nombre_archivo_obstaculos = 'obstaculos.txt'
    obstaculos = cargar_obstaculos(nombre_archivo_obstaculos)
    puntos = obtener_puntos_paso(mapa_lado, obstaculos)

    # Matriz de costes
    costes = 'matriz_distancias.txt'
    matriz_costos = cargar_matriz_desde_txt(costes)

    #Calcular la trayectoria optima
    trayectoria_sin_interpolar = a_estrella(matriz_costos, puntos, inicio, objetivo, heuristica_euclidiana)       
    trayectoria_sin_suavizar = interpolar_camino(trayectoria_sin_interpolar)
    trayectoria = suavizar_trayectoria(trayectoria_sin_suavizar, ventana = 15, orden = 3)


    # Suscribirse al topic de orientacin de la IMU
    rospy.Subscriber("/imu/data", Imu, imu_callback, queue_size=1)
    
    #Imprimir la trayectoria optima
    print("Trayectoria optima:", trayectoria)

    rospy.spin()

if __name__ == '__main__':
    main()
