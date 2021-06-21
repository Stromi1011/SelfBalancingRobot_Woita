#define tr1 3                         //defines for trigger pins
#define tr2 5
#define ech1 2                        //defines for echo pins
#define ech2 4
#define int_r 8                       //defines for obstacle detecded pins
#define int_v 9

int duration;                         //helper variable for the pulse duration
int distance_v;                       //distance to front
int distance_r;                       //distance to back

bool tooClose_v=false;                //flags if its too close to one side
bool tooClose_r=false;

void setup() {
  pinMode(tr1,OUTPUT);                //pinmodes
  pinMode(tr2,OUTPUT);
  pinMode(ech1,INPUT);
  pinMode(ech2,INPUT);
  pinMode(int_v,OUTPUT);
  pinMode(int_r,OUTPUT);
  digitalWrite(int_v,LOW);            //set obstacle detected pins to low -- just to be sure
  digitalWrite(int_r,LOW);
}

void loop() 
{
  digitalWrite(tr1,HIGH);             //distance measurement for side 1
  delayMicroseconds(10);
  digitalWrite(tr1,LOW);

  duration=pulseIn(ech1,HIGH);
  distance_v=0.017*duration;


  digitalWrite(tr2,HIGH);             //distance measurement for side 1
  delayMicroseconds(10);
  digitalWrite(tr2,LOW);

  duration=pulseIn(ech2,HIGH);
  distance_r=0.017*duration;


  if(distance_v<20)                   //if its too close set tooClose flag to 1
  {
    tooClose_v=true;
  }
  if(distance_r<20)
  {
    tooClose_r=true;
  }
  if(distance_v>20)                   //if its not too close set tooClose flag to 0
  {
    tooClose_v=false;
  }
  if(distance_r>20)
  {
    tooClose_r=false;
  }

  digitalWrite(int_v, tooClose_v);    //write the flags (1=high=true) (0=low=false)
  digitalWrite(int_r, tooClose_r);
}
