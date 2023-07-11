import cv2
import numpy as np
import cv2
from cv2 import HoughCircles
import serial
from time import sleep
import math


ser = serial.Serial("COM32", 9600)  # Reemplaza 'COM3' por el puerto correcto
print("Conexión establecida")
nCam = 1  # Valor de la camara, 0 es la del notebook
cap = cv2.VideoCapture(nCam)
cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
cv2.resizeWindow("frame", 640, 480)
cv2.moveWindow("frame", 30, 100)

cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
cv2.resizeWindow("mask", 640, 480)
cv2.moveWindow("mask", 1000, 100)


def load_colors(filename):
    with open(filename, 'r') as file:
        # Lee las líneas del archivo (se espera que haya tres líneas)
        lines = file.readlines()

        # Para cada línea, quita los caracteres innecesarios y divide por los espacios
        color1_hsv = np.array([int(val) for val in lines[0].strip('[]\n').split()])
        color2_hsv = np.array([int(val) for val in lines[1].strip('[]\n').split()])
        color3_hsv = np.array([int(val) for val in lines[2].strip('[]\n').split()])

    return color1_hsv, color2_hsv, color3_hsv


colorBack, colorFront, colorGoal = load_colors("params\colores.txt")

print(colorBack, colorFront, colorGoal)
# H   S   V
LowerColorError = np.array([-10, -50, -50])
UpperColorError = np.array([10, 50, 50])


def enviar_comando(comando):
    ser.write(comando.encode())


enviar_comando("Start")


# Función para dibujar un círculo en la mitad inferior de la imagen
def arrow_pos(img):
    height, width, _ = img.shape
    center_x = width // 2  # Coordenada x del centro del círculo
    center_y = 20  # Coordenada y del centro del círculo
    radius = 27  # Radio del círculo
    color = (0, 255, 0)  # Color del círculo (en formato BGR)
    thickness = 2  # Grosor del círculo
    cv2.circle(img, (center_x, center_y), radius, color, 2)
    cv2.circle(img, (center_x, center_y), 2, (0, 0, 255), 3)
    return center_x, center_y


def detect_circle(image, mask):
    circles = HoughCircles(
        mask,
        cv2.HOUGH_GRADIENT,
        1.2,
        100,
        param1=10,
        param2=10,
        minRadius=20,
        maxRadius=30,
    )

    if circles is not None:
        circles = np.uint16(np.around(circles))
        # print("Se encontró un círculo")
        for i in circles[0, :]:
            # Dibujar el círculo y el centro
            cv2.circle(image, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.circle(image, (i[0], i[1]), 2, (0, 0, 255), 3)
            return i[0], i[1], i[2]
    return None, None, None


def calculate_angle(arrowX, arrowY, xf, yf, xg, yg):
    # Crear los vectores a partir de los pares de puntos
    v1 = np.array([xf - arrowX, yf - arrowY])
    v2 = np.array([xg - arrowX, yg - arrowY])

    # Calcular el producto punto de los dos vectores
    dot_product = np.dot(v1, v2)

    # Calcular la magnitud (longitud) de los dos vectores
    v1_magnitude = np.linalg.norm(v1)
    v2_magnitude = np.linalg.norm(v2)

    # Si alguna de las magnitudes es cero, el ángulo es cero
    if v1_magnitude == 0 or v2_magnitude == 0:
        return 0.0

    # Calcular el coseno del ángulo
    cos_angle = dot_product / (v1_magnitude * v2_magnitude)

    # Calcular el ángulo en radianes
    angle_rad = np.arccos(np.clip(cos_angle, -1.0, 1.0))

    # Convertir el ángulo a grados
    angle_deg = np.degrees(angle_rad)

    # Calcular el vector perpendicular a v1 en sentido horario
    v1_perp = np.array([v1[1], -v1[0]])

    # Calcular el producto escalar entre v1_perp y v2
    dot_product_perp = np.dot(v1_perp, v2)

    # Si el producto escalar es positivo, el ángulo es negativo
    if dot_product_perp > 0:
        angle_deg = -angle_deg

    return angle_deg


# Variable de control para verificar si la ventana está abierta
window_open = True
angle = 90
muestreo = 10
while window_open:
    # Lectura de un cuadro de video
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Dibujo del círculo en el cuadro actual
    arrowX, arrowY = arrow_pos(frame)

    LowerColor1 = colorFront + LowerColorError
    UpperColor1 = colorFront + UpperColorError

    Color1Mask = cv2.inRange(hsv, LowerColor1, UpperColor1)
    Color1Res = cv2.bitwise_and(frame, frame, mask=Color1Mask)

    LowerColor2 = colorBack + LowerColorError
    UpperColor2 = colorBack + UpperColorError

    Color2Mask = cv2.inRange(hsv, LowerColor2, UpperColor2)
    Color2Res = cv2.bitwise_and(frame, frame, mask=Color2Mask)

    LowerColor3 = colorGoal + LowerColorError
    UpperColor3 = colorGoal + UpperColorError

    Color3Mask = cv2.inRange(hsv, LowerColor3, UpperColor3)
    Color3Res = cv2.bitwise_and(frame, frame, mask=Color3Mask)

    xf, yf, radiusf = detect_circle(frame, Color1Mask)
    # xb, yb, radiusb = detect_circle(frame, Color2Mask)
    xg, yg, radiusg = detect_circle(frame, Color3Mask)

    if xg is not None and yg is not None and xf is not None and yf is not None:
        angle = calculate_angle(arrowX, arrowY, xf, yf, xg, yg)
        # print("Ángulo: ", angle)

    res = Color1Res + Color3Res + Color2Res

    cv2.imshow("frame", frame)
    cv2.imshow("mask", res)

    if angle > 5:  # Tecla "D"
        enviar_comando("D")
        print("Derecha")
        muestreo = 0
    elif angle < -5:  # Tecla "I"
        enviar_comando("I")
        print("Izquierda")
        muestreo = 0

    muestreo += 1
    # Salir del bucle si se cierra la ventana
    if cv2.waitKey(1) & 0xFF == ord("q"):
        window_open = False

# Liberar los recursos
cap.release()
cv2.destroyAllWindows()
ser.close()
print("Conexión cerrada")
