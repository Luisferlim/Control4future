#include <Wire.h>
#include <AccelStepper.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

#define tipoInterfaceMotor 1
#define pinoDirecao 2
#define pinoPasso 15

AccelStepper meuStepper(tipoInterfaceMotor, pinoPasso, pinoDirecao);

TaskHandle_t control;

double setPoint = 0.02;
double kp = 3, ki = 0, kd = 0;
double prevError = 0.0;
double integral = 0.0;
unsigned long lastTime = 0;
double u;

double accX, accY, accZ, angle, previosAngle;
bool flag = true;
double buffer[5], contador = 0;

void setup() {
  Serial.begin(57600);

  /*
  Wire.setClock(400000);
  */
  Wire.begin();
  /*
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  */
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  
  meuStepper.setMaxSpeed(10000);  
  meuStepper.setAcceleration(100000);

  xTaskCreatePinnedToCore(
    pidControl,
    "pidControl",
    5000,
    NULL,
    configMAX_PRIORITIES - 1,
    &control,
    0
    );
}

void loop() {
  if(u = 0){
    meuStepper.stop();
  }
  meuStepper.runSpeed();
}

void pidControl(void* parameters){
  while(true){
    mpu6050.update();
    unsigned long now = millis();
    double dt = (now - lastTime);
    lastTime = now;
    int eita = mpu6050.getAngleZ();
    Serial.println(eita);
    int error =  eita - setPoint;

    integral += error * ki;

    int proportional = kp * error;

    int derivative;

    if (dt > 0) {
      derivative = (error - prevError) / dt * kd;
    } else {
      derivative = 0;
    }
    prevError = error;

    u = proportional + integral + derivative;
    u = constrain(u, -90, 90);

    if(u > 0){
      u = map(u, 0, 90, 100, 1000);
    }else{
      u = map(u, -90, 0, -1000, -100);
    }
    
    meuStepper.setSpeed(u);

    
    vTaskDelay(1);
  }
}

/*
void gyro_signals(){
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6);
  int16_t accXLBS = Wire.read() << 8 | Wire.read();
  int16_t accYLBS = Wire.read() << 8 | Wire.read();
  int16_t accZLBS = Wire.read() << 8 | Wire.read();
  
  accX = (double)accXLBS/16384;  
  accY = (double)accYLBS/16384;
  accZ = (double)accZLBS/16384;

  
  
  if(flag){
  angle = atan2(accY, sqrt(accZ*accZ + accX*accX)) * 180 / 3.14159;
   //previousAngle = angle;
  }
  else{
    // if(abs(angle - previousAngle) > 5){
      angle = atan2(accY, sqrt(accZ*accZ + accX*accX)) * 180 / 3.14159;
    }
  } 
}

double mediaMovel(double novoValor){
  
  if(contador<4){
    buffer[contador] = novoValor;
    contador++;
  }
}
*/
void oneStep(int u){
}