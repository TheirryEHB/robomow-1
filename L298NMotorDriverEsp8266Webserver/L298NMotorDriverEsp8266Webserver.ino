
int oldSpeed = 0;
int goSpeed= 150; // Snelheid motor 1
int goSpeed2 = 255; // Snelheid motor 2
const int motor_offset = 5;

/* Motor A connections*/
int enA = 2;
int in1 = 3;
int in2 = 4;
/* Motor B connections*/
int enB = 5;
int in3 = 6;
int in4 = 7;
int relaisPin = 8; //Pin naar relay voor grasmaaier bladen
int relaisStatus = 0; //Status relay

/*Bluetooth module*/
 int data = 0; //Data van bluetooth module

/* Wheel Encoder*/
int wheelPin1 = 9; //Pin voor data wiel 1 (met orange pin draad)
int wheelPin2 = 10; //(met grijze pin draad)
int wheelPings1 = 0; // Aantal pings wiel 1
int wheelPings2 = 0; 
int wheelPingsWhenStopped1 = 0; // Aantal pings wanneer wiel 1 is gestopt
int wheelPingsWhenStopped2 = 0; 
int wheelData1 = 2; // Data weelencoder wiel 1
int wheelData2 = 2; 
int oldWheelData1 = 2; // Oude wieldata wiel 1 om zo te weten wanneer er nieuwe data binnen is gekomen
int oldWheelData2 = 2;
int distance1 = 0; // distance traveled by Robomow (for now: since activation)
int distance2 = 0;
const int R = 9; //Radius of wheel
const int N = 119; //Gemiddelde pings per rotatie
const float pi = 3.14;

// PeriSensor
int periDataFront = 0; // Data voor perimeter sensor aan de voorkant
int periDataRight = 0; // Data voor perimeter sensor aan de rechterkant
int periPinFront = A0; // Pin voor perimeter sensor aan de voorkant
int periPinRight = A1; // Pin voor perimeter sensor aan de rechterkant
int turning = 0;

//Battery
const int analogInPin = A2; // Pin lezen batterij

void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.setTimeout(1000);
  // Serial.println();
  // Serial.print("Configuring access point...");

   // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(relaisPin, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(relaisPin, LOW);

  // Get pings from wheel encoder
  digitalWrite(wheelPin1, INPUT);
  digitalWrite(wheelPin2, INPUT);

  // Get data from peri sensor
  // digitalWrite(periPin, INPUT);

  // goForward(goSpeed, goSpeed2);
  // rotate90Left(goSpeed, goSpeed2);
  

}

void loop() {
 
  float batteryVoltage = getBatteryStatus();

  readEncoder();
  readPerimeters();
  stopRotate90Left();

  if(Serial.available() > 0) { // Checks whether data is comming form the serial port
    data = Serial.read(); //Reads the data from the serial port
    
      if(data == '1'){
        rotateLeft(goSpeed, goSpeed2);
      }
      else if(data == '2'){
        goForward(goSpeed, goSpeed2);
      }
      else if(data == '3'){
        rotateRight(goSpeed, goSpeed2);
      }
      else if(data == '4'){
        goBackward(goSpeed, goSpeed2);
      }
      else if(data == '5'){
        goForward(goSpeed, goSpeed2);
      }
      else if(data == '6'){
        Stop();
      }
      else if(data == '7'){
        rotate90Left(goSpeed, goSpeed2);
      }
      else if(data == '8'){
        driveStraight(100, goSpeed);
      }
  }

}

void driveStraight(int dist, int power){

  setValuesToZero();

  unsigned long num_ticks_l;
  unsigned long num_ticks_r;

  // Set initial motor power
  int power_l = power;
  int power_r = power;

  // Remember previous encoder counts
  unsigned int enc_l_prev = wheelPings1;
  unsigned int enc_r_prev = wheelPings2;

  // Used to determine which way to turn to adjust
  unsigned long diff_l;
  unsigned long diff_r;
  unsigned long target_count = getPingsDistance(dist);
  Serial.println(String(target_count));
  //Wiel 1 is r

  while((wheelPings2 < target_count)){
    readEncoder();
    num_ticks_l = wheelPings1;
    num_ticks_r = wheelPings2;

    Serial.println("w1: " + String(wheelPings1));
    Serial.println("w2: " + String(wheelPings2));
    //Drive
    goForward(power_l, power_r);

    // Number of ticks counted since last time
    diff_l = num_ticks_l - enc_l_prev;
    diff_r = num_ticks_r - enc_r_prev;

    // Store current tick counter for next time
    enc_l_prev = num_ticks_l;
    enc_r_prev = num_ticks_r;

  // If left is faster, slow it down and speed up right
    if ( diff_l > diff_r ) {
      power_l -= motor_offset;
      power_r += motor_offset;
    }

    // If right is faster, slow it down and speed up left
    if ( diff_l < diff_r ) {
      power_l += motor_offset;
      power_r -= motor_offset;
    }

    // Brief pause to let motors respond
    delay(20);
  }

  // Brake
  Stop();
}

/* 
  Get data from perimeter pins and put them in periDataFront and periDataRight
*/
void readPerimeters(){
   periDataFront = analogRead(periPinFront);
  //  Serial.println("SensorDataFront: " + String(periDataFront));
   periDataRight = analogRead(periPinRight);
  //  Serial.println("SensorDataRight: " + String(periDataRight));
   if(periDataFront >= 2000){ // Als voorkant rond de 5cm van de perimeter is --> stop
     Stop();
   }
}

///////////////////////////////////////////// Wheel Encoder part
/* 
  Get data from wheelencoder pins and put them in wheelData1 and wheelData2.
  Also increase wheelpings1 and 2 and wheelPingsWhenStopped1 and 2 if needed.

*/
void readEncoder(){
  wheelData1 = digitalRead(wheelPin1);
  wheelData2 = digitalRead(wheelPin2);

  if(wheelData1 != oldWheelData1){
    oldWheelData1 = wheelData1;
    ++wheelPings1;
    ++wheelPingsWhenStopped1;
    // Serial.println("Wheeldata: " + String(wheelData));
    // Serial.println("Them wheels: " + String(getCounter1()));
  }
  if(wheelData2 != oldWheelData2){
    oldWheelData2 = wheelData2;
    ++wheelPings2;
    ++wheelPingsWhenStopped2;
    // Serial.println("Wheeldata: " + String(wheelData));
    // Serial.println("Them wheels: " + String(getCounter2()));
  }
}

/*
  Return the number of pings foor wheel 1
*/
int getCounter1(){
  int result;
  noInterrupts();
  result = wheelPings1;
  interrupts();
  return result;
}
/*
  Return the number of pings foor wheel 2
*/
int getCounter2(){
  int result;
  noInterrupts();
  result = wheelPings2;
  interrupts();
  return result;
}

/*
  Return distance traveled by wheel 1 in cm
  */
int getDistance1(){
  distance1 = ((2*pi*R)/N) * getCounter1();
  return distance1;
}
/*
  Return distance traveled by wheel 2 in cm
  */
int getDistance2(){
  distance2 = ((2*pi*R)/N) * getCounter2();
  return distance2;
}

long getPingsDistance(int dis){
  return dis/((2*pi*R)/N);
}

///////////////////////////////////////////////////////////////////////// Movement part

/* 
  Keep speed of wheel 1 in allowed range
  */
int speedVerify1(int speed) {
  if (speed < 0){
    speed = 0;
  }
  if (speed > 255){
    speed = 255;
  }
  return speed;
}
/* 
  Keep speed of wheel 2 in allowed range
  */
int speedVerify2(int speed1, int speed2){
   if (speed2 < 0){
    speed2 = 0;
  }
  if (speed2 > 255){
    speed2 = 255;
  }
  return speed2;
}


void goForward(int motorSpeed1, int motorSpeed2){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  motorSpeed1 = speedVerify1(motorSpeed1);
  motorSpeed2 = speedVerify2(motorSpeed1, motorSpeed2);
  analogWrite(enA, motorSpeed1);
  analogWrite(enB, motorSpeed2);   
  // Serial.print("forward:");
  // Serial.println(motorSpeed1);
  // Serial.println(motorSpeed2);
  
}

void goBackward(int motorSpeed1, int motorSpeed2){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  motorSpeed1 = speedVerify1(motorSpeed1);
  motorSpeed2 = speedVerify2(motorSpeed1, motorSpeed2);
  analogWrite(enA, motorSpeed1);
  analogWrite(enB, motorSpeed2);
  // Serial.print("backward:");
  // Serial.println(motorSpeed);
}
void goRight(int motorSpeed1, int motorSpeed2){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  motorSpeed1 = speedVerify1(motorSpeed1);
  motorSpeed2 = speedVerify2(motorSpeed1, motorSpeed2);
  analogWrite(enA, motorSpeed1);
  analogWrite(enB, motorSpeed2/2);
  // Serial.print("right:");
  // Serial.println(motorSpeed);
  
}

void rotateRight(int motorSpeed1, int motorSpeed2){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  motorSpeed1 = speedVerify1(motorSpeed1);
  motorSpeed2 = speedVerify2(motorSpeed1, motorSpeed2);
  analogWrite(enA, motorSpeed1);
  analogWrite(enB, motorSpeed2);
  // bv
  
}


void goLeft(int motorSpeed1, int motorSpeed2){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
  analogWrite(enA, motorSpeed1/2);
  analogWrite(enB, motorSpeed2);
  // Serial.print("left:");
  // Serial.println(motorSpeed1);
  
}
void rotateLeft(int motorSpeed1, int motorSpeed2){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
  analogWrite(enA, motorSpeed1);
  analogWrite(enB, motorSpeed2);
  // Serial.print("rotate left:");
  // Serial.println(motorSpeed1);
  
}

void stopRotate90Left(){
  if(turning == 1){
    if(wheelPingsWhenStopped2 == 90){
      turning = 0;
      Stop();
    }
  }
}

void rotate90Left(int motorSpeed1, int motorSpeed2){
  setValuesToZero();
  turning = 1;
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
  analogWrite(enA, motorSpeed1);
  analogWrite(enB, motorSpeed2);
  // Serial.print("rotating 90 degrees left...");
  // Serial.println(motorSpeed1);
}

void Stop(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  // getDistance1();
  // Serial.println("stop");
  // Serial.println("wheelpings1: " + String(wheelPingsWhenStopped1));
  // Serial.println("wheelpings2: " + String(wheelPingsWhenStopped2));
  setValuesToZero();
}

void setValuesToZero(){
  oldWheelData1 = 2;
  wheelPings1 = 0;
  oldWheelData2 = 2;
  wheelPings2 = 0;
  wheelPingsWhenStopped1 = 0;
  wheelPingsWhenStopped2 = 0;
}

/*
  Set mower to spinning of not spinning
  */
void setMower(bool value) {
  if (value == true) {
    digitalWrite(relaisPin, HIGH);
    relaisStatus = 1;
  }
  if (value == false) {
    digitalWrite(relaisPin, LOW);
    relaisStatus = 0;
  }
}

/*
  Get battery status in volts
  */
float getBatteryStatus() {
  int sensorValue = analogRead(analogInPin);
  float batteryVoltage=map(sensorValue,0,932,0,1680);
  batteryVoltage = batteryVoltage/100;
  return batteryVoltage;
}
