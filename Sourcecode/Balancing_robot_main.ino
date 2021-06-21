#include <MPU6050.h>
#include <Wire.h>

//____________________________________________________-defines
//*************************PIDConsts
#define Kp 5.26                       
#define Ki 10.00
#define Kd 63.90
//*************************/PIDConsts

//**************************motor pins
#define pinStep_1 5                                   //also PORTD bit 6
#define pinDirection_1 2                              //also PORTD bit 3

#define pinStep_2 6                                   //also PORTD bit 7
#define pinDirection_2 3                              //also PORTD bit 4
//**************************/motor pins

//***************************hcsr-04
#define pinObst_1 13
#define pinObst_2 12
//**************************/hcsr-04
//__________________________________________________-globVars
//**********************mpu
MPU6050 mpu;                                          //initialise a MPU6050 element
const float timeStep=0.005;                           //time step for readMPU() (5ms)
volatile double angle = 0;                            //variable for most recent angle
//*********************/mpu

//********************pid
double setpoint=0;                                    //setpoint for desired angle
double PID_P, PID_I, PID_D;                           //variables for PID func
double PID_P_old;                                     //variable to safe last PID result
volatile double PID_out;                              //variable for PID return
//*******************/pid

//**********************motors
int motDelay;                                         //delay used to controll motor speed
//*********************/motors

//*********************hc05
volatile char recByte=0;                              //recieved byte from UART (HC-05)
//********************/hc05


//________________________________________________func

//initializes Timer Interrupt (5ms)
void initTimer()
{
  cli();                                                //disable interrupts

  TCCR1A =0;                                            //prescaler Registers
  TCCR1B=0;
  OCR1A=9999;                                           //output compare register
  TCCR1B |= 0b00001010;                                 
  TIMSK1 |= 0b00000010;                                 //interrupt mask register

  sei();                                                //enable interrupts
} 

//reads Y angle from MPU6050
void readMPU()
{
  Vector readings = mpu.readNormalizeGyro();          //read from gyro
  angle = angle + (double)readings.YAxis*timeStep;    //calculate change in angle
}

//calculates PID
void calcPid(double inpAngle)
{
  PID_P=setpoint-inpAngle;                            //calculate P
  PID_I += PID_P*5;                                   //calculate I
  PID_D = (PID_P - PID_P_old)/5;                      //calculate D

  PID_out= Kp*PID_P + Ki*PID_I + Kd*PID_D;            //calculate PID

  PID_P_old=PID_P;                                    //safe PID for next iteration
}

//modified version of map() 
//now uses double
double scal(double x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//_______________________________________________________________________________-setup
void setup()
{
  pinMode(pinStep_1,OUTPUT);                                    //pinmodes for Motor pins
  pinMode(pinStep_2,OUTPUT);
  pinMode(pinDirection_1,OUTPUT);
  pinMode(pinDirection_2,OUTPUT);
  pinMode(pinObst_1,INPUT);                                     //pinmodes for obstacle detection pins
  pinMode(pinObst_2,INPUT);

  Serial.begin(9600);                                           //start UART for HC-05

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))    //init MPU
  {
    delay(500);
  }
  mpu.calibrateGyro();                                          //calibrate to eliminate static error
  mpu.setThreshold(3);                                          //set sensibility
 
  readMPU();                                                    //read gyro for initial value
  initTimer();                                                  //setup timerinterrupt 1
  PORTD=PORTD | 0x08;                                           //initial mot direction --important for first direction check later
}

//_____________________________________________________________-loop
void loop() 
{
  if((angle<0) && ((PORTD & 0x0C)==0x08))                         //check if direction is set wrong
  {
    PORTD = PORTD & ~0x0C;                                        //if so clear PORTD bit 2 and 3...
    PORTD =  PORTD | 0x04;                                        //...and set correct
  }
  else if((angle>0) && ((PORTD & 0x0C)==0x04))                    //same for other direction
  {
    PORTD = PORTD & ~0x0C;
    PORTD = PORTD | 0x08;
  }

  motDelay=scal(abs(PID_out),0,30*Kp,2900,800);                 //scale pid_out for motors
  motDelay=constrain(motDelay,800,2900);                        //making sure the value is acceptable
 

  if (((angle>1) && (angle<45)) || ((angle<-1) && (angle>-45))) //check if angle is within deadzones
  {
     PORTD =PORTD | 0x60;                                       //set step pins high
    delayMicroseconds(2);                                       //delay > 1us for a4988
    PORTD =PORTD & ~0x60;                                       //set step pins low
    delayMicroseconds(motDelay);                                //delay with preveously calculated delay
  }
   
  recByte=Serial.read();                                        //read UART
  if(recByte=='v' || recByte=='r')                              //check if its an acceptable signal
  {
    if(recByte=='v' && !digitalRead(pinObst_1))                 //check if its forward and HCSR-04 detects obstacles in that direction
    {
      PORTD = PORTD & ~0x0C;                                    //set direction
      PORTD =  PORTD | 0x04;
      for (int i=0;i<400;i++)                                   //send a set ammount of step pulses
      {
        PORTD =PORTD | 0x60;                                    //set step pins high
        delayMicroseconds(2);                                   //delay > 1us for a4988
        PORTD =PORTD & ~0x60;                                   //set step pins low
        delayMicroseconds(800);                                 //delay on the low end
      }
    }
    else if(recByte=='r' && !digitalRead(pinObst_2))            //check if its backwards and HCSR-04 detects obstacles in that direction
    {
      PORTD = PORTD & ~0x0C;                                    //set direction
      PORTD = PORTD | 0x08;  
      for (int i=0;i<400;i++)                                   //send a set ammount of step pulses
      {
        PORTD =PORTD | 0x60;                                    //set step pins high
        delayMicroseconds(2);                                   //delay > 1us for a4988
        PORTD =PORTD & ~0x60;                                   //set step pins low
        delayMicroseconds(800);                                 //delay on the low end
      }
    }
  }
}


//__________________________________________________________________________isr
ISR(TIMER1_COMPA_vect)
{
  sei();                                                      //allow other intterupts...                          --- initially for millies function --should no longer be needed but if we remove it the progran doesnt work !DONT TOUCH!
  TIMSK1 = TIMSK1 & ~0x02;                                    //...but disable this one

  readMPU();
  calcPid(angle);

  TCNT1=0;                                                    //reset timer value
  TIMSK1 = TIMSK1 | 0x02;                                     //reactivate timer1 interrupt
}
