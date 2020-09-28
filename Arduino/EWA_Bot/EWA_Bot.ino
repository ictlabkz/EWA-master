
#include <AFMotor.h>  // Подключаем библиотеку для работы с шилдом 

#include <SoftwareSerial.h>

// Подключаем моторы к клеммникам M2, M3
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);

int state;



void setup() {
  // Задаем максимальную скорость вращения моторов (аналог работы PWM) 
  motor2.setSpeed(255);
  motor2.run(RELEASE);
  motor3.setSpeed(255);
  motor3.run(RELEASE);
  Serial.begin(9600);
}

int i;

void loop() {
 
  if(Serial.available()>0){     
      state = Serial.read();   
    }


    if (state == 'F') {
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor2.setSpeed(255); 
  motor3.setSpeed(255);   
    }

    if (state == 'B') {
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor2.setSpeed(255); 
  motor3.setSpeed(255); 
    }

 
}
