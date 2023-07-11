# Se importan las librerías necesarias
import numpy as np
import cv2

# Se establece el valor de la cámara, 0 es la cámara del notebook
nCam = 1

# Se crea un objeto VideoCapture para capturar video de la cámara especificada
cap = cv2.VideoCapture(nCam) 

# Se definen las matrices de numpy con valor de color inicial de 0 para las variables color1_hsv, color2_hsv y color3_hsv
color1_hsv = np.array([0,0,0])
color2_hsv = np.array([0,0,0])
color3_hsv = np.array([0,0,0])

# Se definen los rangos de colores permitidos para la detección de objetos
LowerColorError = np.array([-5,-30,-30]) 
UpperColorError = np.array([5,30,30])  

info_text = f"Seleciona el primer color con un click"
info_text2 = ''


# Se inicializa la variable nClick en 1
nClick = 1

# Se define una función que maneja los eventos del mouse
def _mouseEvent(event, x, y, flags, param):
    # Se declaran las variables globales que se van a utilizar
    global nClick
    global color1_hsv
    global color2_hsv
    global color3_hsv
    global info_text
    global info_text2
    # Si se presiona el botón izquierdo del mouse
    if event == cv2.EVENT_LBUTTONDOWN:
        # Se convierte el frame capturado a formato HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if(nClick == 1):
            # Si es el primer click se guarda el color seleccionado en la variable color1_hsv
            color1_hsv = hsv_frame[y,x]
            print("Color 1: ",color1_hsv )
            print("Eje x: ", x)
            print("Eje y: ",y)
            info_text = f"Seleciona el segundo color con un click"

            nClick += 1
        elif(nClick == 2):
            # Si es el segundo click se guarda el color seleccionado en la variable color2_hsv
            color2_hsv = hsv_frame[y,x]
            print("Color 2: ",color2_hsv )
            print("Eje x: ", x)
            print("Eje y: ",y)
            info_text = f"Seleciona el tercer color con un click"
            nClick += 1
        elif(nClick == 3):
            # Si es el tercer click se guarda el color seleccionado en la variable color3_hsv
            color3_hsv = hsv_frame[y,x]
            print("Color 3: ",color3_hsv )
            print("Eje x: ", x)
            print("Eje y: ",y)
            nClick += 1
            info_text1 = 'Has seleccionado los tres colores'
            info_text2 = "[Has click para reiniciar]"
        else:
            # Si se han seleccionado tres colores, se reinicia la variable nClick a 1
            info_text = "Seleciona el primer color con un click"
            info_text2 = ''
            color1_hsv = np.array([0,0,0])
            color2_hsv = np.array([0,0,0])
            color3_hsv = np.array([0,0,0])
            nClick = 1

# Se crea una ventana de visualización para la imagen original y la imagen segmentada de los tres colores
cv2.namedWindow('Imagen Original',  cv2.WINDOW_NORMAL )   
cv2.resizeWindow('Imagen Original', 640, 480)
cv2.moveWindow('Imagen Original', 30, 100)

cv2.namedWindow('Imagen Segmentada',  cv2.WINDOW_NORMAL)
cv2.resizeWindow('Imagen Segmentada', 640, 480)    
cv2.moveWindow('Imagen Segmentada', 700, 100)

# Se establece la función que maneja los eventos del mouse
cv2.setMouseCallback('Imagen Original',_mouseEvent)
            
while(True):
    # Se lee un frame de la cámara
    ret, frame = cap.read()    
    
    # Se convierte el frame capturado a formato HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Se definen los límites de color permitidos para la detección de objetos para los tres colores
    LowerColor1 = color1_hsv + LowerColorError
    UpperColor1 = color1_hsv + UpperColorError    
    
    LowerColor2 = color2_hsv + LowerColorError
    UpperColor2 = color2_hsv + UpperColorError    
    
    LowerColor3 = color3_hsv + LowerColorError
    UpperColor3 = color3_hsv + UpperColorError    
    
    # Se aplican máscaras de color para detectar objetos para los tres colores
    Color1Mask = cv2.inRange(hsv, LowerColor1, UpperColor1)
    Color1Res = cv2.bitwise_and(frame,frame, mask= Color1Mask)

    Color2Mask = cv2.inRange(hsv, LowerColor2, UpperColor2)
    Color2Res = cv2.bitwise_and(frame,frame, mask= Color2Mask)

    Color3Mask = cv2.inRange(hsv, LowerColor3, UpperColor3)
    Color3Res = cv2.bitwise_and(frame,frame, mask= Color3Mask)

    # Se suman las imágenes segmentadas de los tres colores en una sola imagen
    SumRes = cv2.add(Color1Res, Color2Res)
    SumRes = cv2.add(SumRes, Color3Res)

    # Se crea un texto con la información de los colores seleccionados
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.7
    thickness = 2
    color = (0, 0, 255)  # Color en formato BGR (Cyan)
    x_offset = 10
    y_offset = 30
    cv2.putText(frame, info_text, (x_offset, y_offset), font, scale, color, thickness, cv2.LINE_AA)
    cv2.putText(frame, info_text2, (x_offset, y_offset+50), font, scale, color, thickness, cv2.LINE_AA)
        
    # Se muestran las imágenes original y segmentada en las ventanas correspondientes
    cv2.imshow('Imagen Original',frame)    
    cv2.imshow('Imagen Segmentada',SumRes)

    # Si se presiona la tecla ESC, se cierran las ventanas y se libera la cámara
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Se libera la cámara cuando se termina el programa
cap.release()
# Se cierran todas las ventanas
cv2.destroyAllWindows()
