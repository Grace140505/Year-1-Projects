/*-----ULTRASONIC SENSOR AND BLUETOOTH COMBINATION-----*/

//Bluetooth
#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define CIRCLE 'C'
#define CROSS 'X'
#define TRIANGLE 'T'
#define SQUARE 'S'
#define START 'A'
#define PAUSE 'P'
//Motors -> Ultrasonic sensor referenced as head
#define IN1 13 //Right wheel
#define IN2 12 //Right Wheel
#define IN3 2 //Left wheel
#define IN4 A3 //Left wheel
#define ENA 11 //PWM Port //Right wheels
#define ENB 3 //PWM port //Left wheels
//Ultrasonic sensor
#define TRIG A5
#define ECHO A4


/*----------------------------------------------------------------------------------------------*/
//Wheels speed const
const int TARGET_LEFT_SPEED = 160;  //Range: 0-255 (PWM)
const int TARGET_RIGHT_SPEED = 150;  //Range: 0-255 (PWM)
//Time
long int start_time;
//Move straight speed
int leftSpeed = TARGET_LEFT_SPEED;//Updated left speed
int rightSpeed = TARGET_RIGHT_SPEED;//Updated right speed
/*----------------------------------------------------------------------------------------------*/


void setup() {
  //Initialize ultrasonic
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  //Reset motor
  func_motorInit();
  //Begin bluetooth communication
  Serial.begin(9600);

  //Start counting time
  start_time = millis();
}


void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    char command = Serial.read();
    executeCommand(command);
  }
}

//
void executeCommand(char command) {
  switch (command) {
    case FORWARD:{ //Forward
      func_ADAS();
      func_moveForward();
      break;}
    case BACKWARD:{ //Backward
      func_moveBackward();
      break;}
    case LEFT:{ //Smooth turn left
      func_ADAS();
      func_turnLeft(180,100);
      break;}
    case RIGHT:{ //Smooth turn right
      func_ADAS();
      func_turnRight(100,180);
      break;}
    case CIRCLE:{ //Tight right turn
      func_stationaryTurnRight();
      break;}
    case CROSS:
      break;
    case TRIANGLE:
      break;
    case SQUARE:{ //Tight left turn
      func_stationaryTurnLeft();
      break;}
    case START:
      break;
    case PAUSE:
      break;
    default:
      func_stop();
      break;
    
  }
}

//Detect distance of obstacle
int func_getDistance(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(10);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH);//Blocking function
  float dist = duration * 0.034 / 2.0; // Calculte
  return dist;
}

void func_motorInit(){
  //Initialize motor pin
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  //Initialize speed pin
  pinMode(ENA, OUTPUT); 
  pinMode(ENB, OUTPUT);
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}
//Stop
void func_stop(){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  delayMicroseconds(10);
}
//Move forward
void func_moveForward(){
  //Set speed
  analogWrite(ENA,TARGET_RIGHT_SPEED);
  analogWrite(ENB,TARGET_LEFT_SPEED);
  //Set direction
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  delayMicroseconds(10);
}
//Move backward
void func_moveBackward(){
    //Set speed
  analogWrite(ENA,TARGET_RIGHT_SPEED);
  analogWrite(ENB,TARGET_LEFT_SPEED);
  //Set direction
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  delayMicroseconds(10);
}
//Turn smooth turn left
void func_turnLeft(int R, int L){
  //Set speed
  analogWrite(ENA,R);//Right wheels
  analogWrite(ENB,L);//Left wheels
  //Set direction
  digitalWrite(IN1, HIGH);//Right motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);//Left motor forward
  digitalWrite(IN4, LOW);
  delayMicroseconds(10);
}
//Turn smooth turn right
void func_turnRight(int R, int L){
  //Set speed
  analogWrite(ENA,R);//Right wheels
  analogWrite(ENB,L);//Left wheels
  //Set direction
  digitalWrite(IN1, HIGH);//Right motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);//Left motor forward
  digitalWrite(IN4, LOW);
  delayMicroseconds(10);
}
//Stationary turn right
void func_stationaryTurnRight(){
  //Set speed
  analogWrite(ENA,150);//Right wheels
  analogWrite(ENB,150);//Left wheels
  //Set direction
  digitalWrite(IN1, LOW);//Right motor backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);//Left motor forward
  digitalWrite(IN4, LOW);
  delayMicroseconds(10);
}
//Stationary left turn
void func_stationaryTurnLeft(){
  //Set speed
  analogWrite(ENA,150);//Right wheels
  analogWrite(ENB,150);//Left wheels
  //Set direction
  digitalWrite(IN1, HIGH);//Right motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);//Left motor backward
  digitalWrite(IN4, HIGH);
  delayMicroseconds(10);
}
void func_ADAS(){
  int distance = func_getDistance();
  
  if (distance <= 10){
    func_stop();
    delayMicroseconds(5);
    func_moveBackward();
    delay(350);
    func_stationaryTurnRight();
    delay(1000);
  }
  else if ((distance <= 20)&&(distance > 10)){
    func_stationaryTurnRight();
    delay(280);
    func_moveForward();
    delay(700);
    func_stationaryTurnLeft();
    delay(350);
    func_moveForward();
    delay(600);
    func_stationaryTurnLeft();
    delay(380);
    func_moveForward();
    delay(600);
    func_stationaryTurnRight();
    delay(380);
  }

}