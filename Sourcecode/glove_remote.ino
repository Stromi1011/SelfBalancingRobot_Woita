#include <MPU6050.h>


//*********************LEDs
#define outputPin_f 2
#define outputPin_r 13
#define outputPin_l 6
#define outputPin_h 8
//*********************/LEDs

//********************MPU6050
MPU6050 mpu;
const float timeStep = 0.01;
unsigned long timer = 0;

//Accelaration Variables:
float pitch =0;
float roll = 0;
//********************/MPU6050


//_____________________________functions

void readMPU()
{
  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch and Roll:
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;

  // Wait to full timeStep period
  delay((timeStep*1000) - (millis() - timer));
}


void setup() 
{
  //start serial for HC-05
  Serial.begin(9600);
  
  //initialize MPU
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
  
    delay(500);
  }
  //set led pins to OUTPUT
  pinMode(outputPin_f, OUTPUT);
  pinMode(outputPin_h, OUTPUT);
  pinMode(outputPin_r, OUTPUT);
  pinMode(outputPin_l, OUTPUT);
  
  //gyro calibration
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  digitalWrite(outputPin_l, LOW);

}

void loop() {
  //get angles
  readMPU();                                      
  
  //check angles and react accordingly
  if(roll > 10)
  {
    digitalWrite(outputPin_l, HIGH);
    delay(100);
    digitalWrite(outputPin_l, LOW);
  }
  
  else if(roll < -10)
  {
    digitalWrite(outputPin_r, HIGH);
    delay(100);
    digitalWrite(outputPin_r, LOW);
  }

  else if(pitch > 15)
  {
    digitalWrite(outputPin_h, HIGH);      //turn led on
    Serial.write('r');                    //send byte
    delay(100);                           //delay
    digitalWrite(outputPin_h, LOW);       //led off
  }
  
  else if(pitch < -15)
  {
    digitalWrite(outputPin_f, HIGH);
    Serial.write('v');
    delay(100);
    digitalWrite(outputPin_f, LOW);
  
  }
}
