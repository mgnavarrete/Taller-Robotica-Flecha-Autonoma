#include <Servo.h>

Servo servoMotor;  // Crear un objeto de la clase Servo
int servoPin = 9;  // Pin de control del servo
int angulo = 90;   // Ángulo inicial del servo

void setup() {
  servoMotor.attach(servoPin);  // Conectar el servo al pin 9
  Serial.begin(9600);           // Iniciar comunicación serial a 9600 bps
  Serial.println("Listo para recibir comandos");
  servoMotor.write(angulo);     // Posicionar el servo en el ángulo inicial
}

void loop() {
  if (Serial.available() > 0) {
    char comando = Serial.read(); 
    if (comando == 'Start') {          // Restar ángulo
        angulo = 90;
        servoMotor.write(angulo);
      }
     
    else if (comando == 'D') {          // Restar ángulo
      angulo -= 1;
      if (angulo < 0) {
        angulo = 0;
      }
      servoMotor.write(angulo);
    } else if (comando == 'I') {   // Sumar ángulo
      angulo += 1;
      if (angulo > 180) {
        angulo = 180;
      }
      servoMotor.write(angulo);
    }
  }
}
