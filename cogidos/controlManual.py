import serial
import keyboard

# Configuración del puerto serial
ser = serial.Serial("COM32", 9600)  # Reemplaza 'COM3' por el puerto correcto
print("Conexión establecida")


# Función para enviar comandos al Arduino
def enviar_comando(comando):
    ser.write(comando.encode())


enviar_comando("Start")
try:
    while True:
        if keyboard.is_pressed("right"):  # Tecla "D"
            enviar_comando("D")
            print("Restar ángulo")
        elif keyboard.is_pressed("left"):  # Tecla "I"
            enviar_comando("I")
            print("Sumar ángulo")

except KeyboardInterrupt:
    ser.close()
    print("Conexión cerrada")
