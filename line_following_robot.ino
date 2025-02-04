#include<Servo.h>

#define enA 5           //RIGHT
#define enB 6           //LEFT
#define IN1 7
#define IN2 9
#define IN3 13
#define IN4 12

#define ECHO A4
#define TRIG A5

#define SERVO_PWM 11

#define LS  3
#define RS  2 
#define ML  4
#define MR  A3

enum Direction {FORWARD, BACKWARD};
int state_mode;
char command;
uint32_t dist = 0;

enum DriveModes {AUTO, MANUAL};

uint32_t obstacle_count=0;
bool armUp = false;
uint32_t time_elapsed;
uint32_t time_start;
uint32_t right_time_start;
uint32_t leftturn_duration;
uint32_t left_time_start;
uint32_t back_on_line;
uint32_t forward_clk;

DriveModes driveMode = MANUAL;
Servo servo;

void setup()
{
  Serial.begin(9600);
  
  // Initial direction of the robot is forward.
  ML_direction(FORWARD);
  MR_direction(FORWARD);

  // Starting time of the robot.
  time_start = millis();

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
//  pinMode(SERVO_PWM, OUTPUT);
  servo.attach(SERVO_PWM);
  pinMode(LS,INPUT);
  pinMode(RS,INPUT); 
  pinMode(MR,INPUT);
  servo.write(180);
}


void loop()
{
  command = receiveData();

  // delay(500);
  if (command == 'W')
    driveMode = AUTO;
  if (command == 'w')  
    driveMode = MANUAL;
  
  if (driveMode == MANUAL)
    manualMode(command);
  else if (driveMode == AUTO)
    autoMode();  
}

void MR_direction(Direction dir)
{
  if (dir == FORWARD)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else if (dir == BACKWARD)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

void ML_direction(Direction dir)
{
  if (dir == FORWARD)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if (dir == BACKWARD)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

uint32_t check_distance()
{
  uint32_t duration;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  duration = pulseIn(ECHO, HIGH);

  uint32_t distance = 0.034 * duration * 0.5;

  return distance;
}

char receiveData()
{
  //Following Condition is true whenever we send a command from Bluetooth Terminal App
  char rxData;
  
  if (Serial.available()>0)
  {
    //Read the bluetooth data and store it in colorDetect variable using Serial.read() command here
    rxData = Serial.read();
  }

  return rxData;
}

void manualMode(char inpCommand)
{
  switch (inpCommand)
  {
    break;
    case 'F':
      analogWrite(enA, 114);//right
      analogWrite(enB, 115);//left
      ML_direction(FORWARD);
      MR_direction(FORWARD);
    break;
    case 'B':
      analogWrite(enA, 114);//right
      analogWrite(enB, 115);//left
      ML_direction(BACKWARD);
      MR_direction(BACKWARD);
    break;
    case 'L':
      analogWrite(enA, 114);//right
      analogWrite(enB, 115);//left
      ML_direction(BACKWARD);
      MR_direction(FORWARD);
    break;
    case 'R':
      analogWrite(enA, 114);//right
      analogWrite(enB, 115);//left
      ML_direction(FORWARD);
      MR_direction(BACKWARD);
    break;
    case 'G':             //forward left
      analogWrite(enA, 114);//right
      analogWrite(enB, 80);//left
      ML_direction(FORWARD);
      MR_direction(FORWARD);
    break;
    case 'I':           //forward right
      analogWrite(enA, 80);//right
      analogWrite(enB, 115);//left
      ML_direction(FORWARD);
      MR_direction(FORWARD);
    break;
    case 'H':           //back left
      analogWrite(enA, 115);//right
      analogWrite(enB, 80);//left
      ML_direction(BACKWARD);
      MR_direction(BACKWARD);
    break;
    case 'J':     //back right
      analogWrite(enA, 80);//right
      analogWrite(enB, 115);//left
      ML_direction(BACKWARD);
      MR_direction(BACKWARD);
    break;
    default:
      analogWrite(enA, 0);//right
      analogWrite(enB, 0);//left
    break;
  }
}

void turnSpeed()
{
  analogWrite(enA, 100);//right
  analogWrite(enB, 100);//left
}

void straightSpeed()
{
  analogWrite(enA, 114);//right
  analogWrite(enB, 115);//left
}

void halt()
{
  analogWrite(enA,0);
  analogWrite(enB,0);
}


void autoMode()
{
  //Read IR Sensors. if HIGH (BLACK Line) or LOW (WHITE Line).
  bool RS_val= digitalRead(RS);
  bool LS_val= digitalRead(LS);
  bool MR_val= digitalRead(MR);
  // Read Ultrasonic sensor.
  uint32_t current_distance = check_distance();
  Serial.println(current_distance);
  // Line following.
  if(current_distance < 10)
  {
    halt();
    if (obstacle_count == 0) // If distance is less than 10, start obstacle avoidance sequence.
    {
      obstacle_avoidance();
      obstacle_count++;
    }
    else if (obstacle_count >= 1)
    {
      servo_motion();
      obstacle_count++;
    } 
  }
  else if(RS_val == HIGH && LS_val == LOW)
  {
    //Turn RIGHT
    turnSpeed();
    ML_direction(FORWARD);
    MR_direction(BACKWARD);
  }

  else if (RS_val == LOW && LS_val == HIGH)
  {
    //Turn LEFT
    turnSpeed();
    ML_direction(BACKWARD);
    MR_direction(FORWARD);
  }

  else if ((RS_val==LOW && LS_val==LOW) || (RS_val==HIGH && MR_val==HIGH && LS_val==HIGH)) //for the perpendicular line that will be in the middle
  {
    straightSpeed();
    ML_direction(FORWARD);
    MR_direction(FORWARD);
  }
}



void obstacle_avoidance()
{
  left_time_start=millis();
  //turn left
  do
  {
    turnSpeed();
    ML_direction(BACKWARD);
    MR_direction(FORWARD);
  }
  while(check_distance() < 15);   //until the obstacle is in path of the sensor, it keeps turning
  delay(100);
  leftturn_duration=millis()-left_time_start;

  halt();
  delay(100);

  straightSpeed();
  ML_direction(FORWARD);
  MR_direction(FORWARD);
  delay(700);

  //turn right again to get back on line
  right_time_start=millis();
  do{
    turnSpeed();
    ML_direction(FORWARD);
    MR_direction(BACKWARD);
   }
  while(millis()-right_time_start<leftturn_duration);

  halt();
  delay(100);

  do
  {
    straightSpeed();
    ML_direction(FORWARD);
    MR_direction(FORWARD);
  }while(digitalRead(RS)==LOW && digitalRead(LS)==LOW && digitalRead(MR)==LOW);
}
  

 void servo_motion()
 {
  servo.write(90);
  delay(1000);
  servo.write(180);
 }