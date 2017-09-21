//USE WITH THE SINGLE BOARD PIDDYBOT(Complete PCB)
// Latest Version of PIDDYBOT Self Balancing Program 02/12/15
// Program created by Sean Hodgins.
// Http://Idlehandsproject.com
// This is free to be shared, altered, and used.
// Find the Second target angle and tune for your bot, it may be different.
// LIBRARIES
//For use with board Version 1.1 - ATMEGA32U4 with DRV8835 Motor Driver


//#define DEBUG_ENABLE
//#define HARD_PID_VALUES
//#define EPID_TUNE
//#define RAW_ACC
//#define BT_CONN
#define BT_GRAPHS
#include <Wire.h>

#include "Kalman.h"
#include <MPU6050.h>
#include <I2Cdev.h>

#include <EEPROM.h>

//#include <PinChangeInt.h>
#include <Encoder.h>

int A = 1;
int B = 2;

//Motor A
int PWMA = 10; //Speed control
int AIN1 = 9; //Direction

//Motor B
int PWMB = 13; //Speed control
int BIN1 = 5; //Direction

// Potentiometers
static int pot1 = A3;
static int pot2 = A4;
static int pot3 = A5;

static int BATT = A6;

//float distancecount = 0;
float correction = -10;
int correctionEn = 1;
int returnto = 0;

int moveDir = 1;

#define RESTRICT_PITCH


//int drive = 0;

// PID CONSTANTS
float Kp = 0;
float Ki = 0;
float Kd = 0;

// from MPU datasheet
//const float accScale = 8192;
//const float gyroScale = 131;

//const int numGyroSamples = 300;

//const float gyroWeight = .90;
//const float accWeight = .10;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

Kalman kalmanX; // Create the Kalman instances
//Kalman kalmanY;

double gyroXangle;//, gyroYangle; // Angle calculate using the gyro only
double compAngleX;//, compAngleY; // Calculated angle using a complementary filter
double kalAngleX;//, kalAngleY;

uint32_t timer;
uint8_t i2cData[14];
int16_t ax, ay, az;
int16_t gx, gy, gz;

MPU6050 mpu;

float gyroOffset = 0; // set in calibrateGyro()

const int AvgAngles = 3;
float prevTargetAngle = 0;
float targetAngle = -90;
uint8_t saveAngle = 0;
int tAngle = 0;


float angles[5];

float currAngle = 0;
float prevAngle = 0;
float prevAngles[AvgAngles];
int prevAngleI = 0;
int motorSpeed;
float rotate = 1;
int forward = 0;
int coursetiming = 0;
int dir = 0;
int lastdir = 0;

float errorSum = 0;
float currError = 0;
float prevError = 0;
float iTerm = 0;
float dTerm = 0;
float pTerm = 0;

//Location PID CONTROL - These are the PID control for the robot trying to hold its location.
float Lp = 1;
float Li = 0;
float Ld = 0;
float offsetLoc = 0;
float pT, iT, dT = 0;
float errorS = 0;
float prevE = 0;
float mSpeed = 0;
float Movement = 0;

int turnREn = 0;
int turnLEn = 0;

int fallflag = 0;

unsigned long previousBTTime = 0;


const long BTinterval = 1000;
////////////NEW ENCODER////////////////
//#define A_ENC 12
//#define B_ENC
//Encoder myEncA(14, 12);
Encoder myEncB(8, 6);
void pin3func() {
  //  returnto = myEncA.read();

  returnto = constrain(returnto, -5000, 5000);

}
/////////////NEW ENCODER////////////////

void setup() {
  //MOTOR SETUP
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BATT, INPUT);

  // Serial Connection
  Serial.begin(38400);
  Serial1.begin(115200);
  Serial1.flush();
  //while(!Serial1);
  //while(!Serial);
  // IMU Connection
  Wire.begin();

  delay(5);
  mpu.initialize();

  // Testing MPU
#ifdef RESTRICT_PITCH
  Serial1.println(mpu.testConnection() ? "MPU is good!" : "Oops! Check the connections");
#endif

  KalmanInit();

  //  myEncA.write(0);
  myEncB.write(0);
  checkBatt();
  saveAngle = EEPROM.read(1);
  tAngle = saveAngle * -1;
}


void loop() {


  //testMotors();
  checkBatt();
  checkPots();


#ifdef EPID_TUNE
  PIDtune();
#else
  Lp = 0.05;
  Ld = 0.0;
#endif

  updateLocation();
  targetAngle = tAngle + correction;// + (correction * correctionEn * 1);
  //Serial.println(targetAngle);
  updateAngle();
  //Serial.print(targetAngle);
  //Serial.print("\t");
  //Serial.println(currAngle);



#ifdef DEBUG_ENABLE
  //Serial.print("P term: ");
  //Serial.print(Kp);
  //Serial.print("I term: ");
  //Serial.print(Ki);
  //Serial.print("D term: ");
  //Serial.println(Kd);

  Serial.print(currAngle);
  Serial.print(",");
  Serial.print("0");
  Serial.print(",");
  Serial.print("0");
  Serial.print(",");
  Serial.println("0");
#else
#endif



#ifdef HARD_PID_VALUES ///Makes the PID POTs do nothing!Hard values 
  Kp = 7.5;
  Ki = 0.1;
  Kd = 40;
#else
  Kp = map(analogRead(pot1), 0, 1023, 0, 1000);  //Tuning Pots for P I and D term.
  Ki = map(analogRead(pot2), 0, 1023, 0, 100);
  Kd = map(analogRead(pot3), 0, 1023, 0, 5000);
  Kp = Kp / 100;
  Ki = Ki / 100;
  Kd = Kd / 100;
#endif

  updateSpeed();
  updateMotor();

#ifdef BT_GRAPHS
  unsigned long currentBTTime = millis();

  if (currentBTTime - previousBTTime >= 10) {
    // save the last time you blinked the LED
    previousBTTime = currentBTTime;
    Serial1.print("E");
    Serial1.print(currAngle);
    Serial1.print(",");
    Serial1.print(motorSpeed);
    Serial1.print('\n');
  }
#endif



}

//Subroutine for finding new angle and using Kalman Filter.
void updateAngle() {

  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

#ifdef RAW_ACC
  Serial.print("Ax: ");
  Serial.print(accX);
  Serial.print("Ay: ");
  Serial.print(accY);
  Serial.print("Az: ");
  Serial.println(accZ);
  Serial.print("Gx: ");
  Serial.print(gyroX);
  Serial.print("Gy: ");
  Serial.print(gyroY);
  Serial.print("Gz: ");
  Serial.println(gyroZ);
#endif
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  //double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  //double gyroXrate = gyroX / 16.4;

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  }
  else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter


  compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; // Calculate the angle using a Complimentary filter


  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;


  currAngle = compAngleX;

  //currAngle = kalAngleX;
  //int elim = (currAngle - prevAngle);
  //if (elim > 75 || elim < -75){
  // Serial.println("Fault!");
  // currAngle = prevAngle;//while(1);
  //}


  prevAngle = currAngle;
#ifdef BT_CONN
  Serial1.print(roll); Serial1.print(",");
  Serial1.print(gyroXangle); Serial1.print(",");
  Serial1.print(compAngleX); Serial1.print(",");
  Serial1.println(kalAngleX);
#endif
}


//PID Loop for Motor Speed control based on angle
void updateSpeed() {


  float K = 1;
  currError = targetAngle - currAngle;
  int errCheck = (currError - prevError);
  //Serial.print(currError);
  //Serial.print("   ");
  //Serial.print(prevError);
  //Serial.print("   ");
  //Serial.println(errCheck);
  if (abs(errCheck) > 20) {
    updateAngle();
    updateAngle();
    updateAngle();
    updateAngle();
    currError = targetAngle - currAngle;
    errCheck = (currError - prevError);

  }

  currError = constrain(currError, -75, 75);
  //Serial.println(currError);
  pTerm = Kp * currError;
  errorSum += currError;
  errorSum = constrain(errorSum, -200, 200);
  //Serial.println(errorSum);
  iTerm = Ki * errorSum;
  dTerm = Kd * (currError - prevError);
  prevError = currError;

  pT = Lp * returnto;
  dT = Ld * (returnto - prevE);
  prevE = returnto;
  //Serial.println(errorSum);


  if (fallflag == 1) {
    if (currError < 2 && currError > -2
       ) {
      fallflag = 0;
    }
  }

  if (currError > 60 || currError < -60) {
    stop();
    motorSpeed = 0;
    fallflag = 1;
    //    myEncA.write(0);
    myEncB.write(0);
  }
  else {
    if (fallflag == 0) {
      motorSpeed = constrain(K * (pTerm + iTerm + dTerm + pT + dT), -255, 255);
      //motorSpeed = constrain(K*(pTerm + iTerm + dTerm), -255, 255);
    }
  }
  //Serial.println(motorSpeed);

}


void updateMotor() {
  ///////Dont need now with encoders////////////////
  if (motorSpeed < 0) {
    dir = 1;
  }
  else if (motorSpeed > 0) {
    dir = 0;
  }

  if (motorSpeed < 0) {
    motorSpeed = motorSpeed * -1;
  }

  move(1, motorSpeed, dir, rotate);
  move(2, motorSpeed, dir, rotate);
  if (motorSpeed == 0) {
    stop();
  }
}


void move(int motor, int speed, int direction, float turn) {
  //Move specific motor at speed and direction
  //motor: 0 for B 1 for A
  //speed: 0 is off, and 255 is full speed
  //direction: 0 clockwise, 1 counter-clockwise


  int leftspeed, rightspeed;
  leftspeed = constrain((speed - (turn * turnREn)) , 0, 255);
  rightspeed = constrain((speed - (turn * turnLEn)) , 0, 255);


  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  /*if (lastdir != direction){
    stop();
    }
    lastdir = direction;
  */

  if (motor == 1) {
    digitalWrite(AIN1, inPin1);
    analogWrite(PWMA, leftspeed);
  }
  else {
    digitalWrite(BIN1, inPin1);
    analogWrite(PWMB, rightspeed);
  }
}
//Turns motors off.
void stop() {
  //Enable Standby
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

}

//Determines distance from target and sets an angle to make robot return to position.
void updateLocation() {
  //returnto = newPosition / 10;
  long AE, BE = 0;
  //  AE = myEncA.read();
  BE = myEncB.read();

  BE = BE * -1;
  if (BE > 400) {
    myEncB.write(-400);
  }
  //  if (AE > 400) {
  //    myEncA.write(400);
  //  }
  if (BE < -400) {
    myEncB.write(400);
  }
  //  if (AE < -400) {
  //    myEncA.write(-400);
  //  }
  //Serial.print(AE);
  //Serial.print(", ");
  //Serial.println(BE);
  offsetLoc = BE;
  //offsetLoc = (returnto + Movement) / 10 ;
  pT = Lp * offsetLoc;
  //errorS += offsetLoc;
  //errorS = constrain(errorS, -200, 200);
  //Serial.println(errorS);
  //iT = Li * errorS;
  dT = Ld * (offsetLoc - prevE);
  prevE = offsetLoc;
  correction = constrain(pT + dT, -4, 4);

}


char dataIn = 'S';
int repeat = 1;


/*///BLUETOOTH USE - NOT NEEDED
  void checkSerial(){
  char determinant;

  char lastData;
  int Movespeed = 7;
  float incrementAmt = 10;
  float maxDist = 100;


  if (Serial1.available() > 0)    //Check for data on the serial lines.
  {
    dataIn = Serial1.read();  //Get the character sent by the phone and store it in 'dataIn'.

    if (dataIn == 'S' || dataIn == 'F' || dataIn == 'B' || dataIn == 'L' || dataIn == 'R' || dataIn == 'I' || dataIn == 'J' || dataIn == 'G' || dataIn == 'H' ) {
      //Serial.println(dataIn);
      if (dataIn != 'S') {
        //myEncA.write(0);
        //myEncB.write(0);
      }
      if (dataIn == 'F')
      {
        determinant = 'F';

        if (Movement < maxDist) {
          Movement += incrementAmt;
        }
        rotate  = 0;

      }
      else if (dataIn == 'B')
      {
        determinant = 'B';
        //drive = Movespeed * -1;
        //correction = correction - 0.01;
        if (Movement > maxDist * -1) {
          Movement += incrementAmt * -1;
        }
        rotate  = 0;
      }
      else if (dataIn == 'L')
      {
        determinant = 'L';
        rotate = 20;
        turnLEn = 1;
        turnREn = -1;
      }
      else if (dataIn == 'R')
      {
        determinant = 'R';
        rotate = 20;
        turnREn = 1;
        turnLEn = -1;
      }
      else if (dataIn == 'I')
      {
        determinant = 'I';

        if (Movement < maxDist) {
          Movement += incrementAmt;
        }

        rotate = 20;
        turnREn = 1;
        turnLEn = -1;
      }
      else if (dataIn == 'J')
      {
        determinant = 'J';

        if (Movement > maxDist * -1) {
          Movement += incrementAmt * -1;
        }
        rotate = 20;
        turnREn = 1;
        turnLEn = -1;

      }
      else if (dataIn == 'G')
      {
        determinant = 'G';

        if (Movement < maxDist) {
          Movement += incrementAmt;
        }

        rotate = 20;
        turnLEn = 1;
        turnREn = -1;
      }
      else if (dataIn == 'H')
      {
        determinant = 'H';

        if (Movement > maxDist * -1) {
          Movement += incrementAmt * - 1;
        }
        rotate = 20;
        turnLEn = 1;
        turnREn = -1;
      }
      else if (dataIn == 'S')
      {

        //Serial.println(repeat);
        repeat = 1;
        if (Movement > 0) {
          Movement = Movement - 1;
        }
        determinant = 'S';
        //drive = 0;
        rotate = 0;
        //returnto = 0;
        correctionEn = 1;
        turnLEn = 0;
        turnREn = 0;
      }

      if (lastData != dataIn)
        repeat = 0;

      lastData = dataIn;

    }

  }




  }
*/

///Check the battery voltage, and stop system running if low.
void checkBatt() {
  int voltage = 0;
  voltage = analogRead(BATT);



  //Serial.println(voltage);
  if (voltage < 200) {
    Serial.println("Recharge Battery");
    while (1) {
      digitalWrite(17, HIGH);
      delay(500);
      digitalWrite(17, LOW);
      delay(500);
    }
  }
}


//If all POTs set to Zero, set the current angle to target angle. This will allow user to change the taget angle for better balancing.
void checkPots() {
  int P1, P2, P3 = 0;
  int pT = 50;
  int zeroed = 0;
  P1 = analogRead(pot1);
  P2 = analogRead(pot2);
  P3 = analogRead(pot3);
  while ((P1 < pT) && (P2 < pT) && (P3 < pT)) {
    updateAngle();
    tAngle = currAngle;
    P1 = analogRead(pot1);
    P2 = analogRead(pot2);
    P3 = analogRead(pot3);
    zeroed = 1;
  }
  saveAngle = tAngle * -1;
  if (zeroed == 1) {
    uint8_t tempAngle = EEPROM.read(1);
    if (tempAngle != saveAngle) {
      EEPROM.write(1, saveAngle);
      Serial.println("Saving to EEPROM");
    }
    zeroed = 0;
  }

}
/*
  void PIDtune(){


  if (Serial1.available() > 0) {   //Check for data on the serial lines.

    dataIn = Serial1.read();  //Get the character sent by the phone and store it in 'dataIn'.

    if (dataIn == 'P')
    {
      Lp += 0.01;
      Serial1.println(Lp);
    }


    else if (dataIn == 'L')
    {
      Lp += -0.01;
      Serial1.println(Lp);
    }
    else if (dataIn == 'I')
    {
      Li += 0.01;
      Serial1.println(Li);
    }
    else if (dataIn == 'J')
    {
      Li += -0.01;
      Serial1.println(Li);
    }
    else if (dataIn == 'D')
    {
      Ld += 0.01;
      Serial1.println(Ld);
    }
    else if (dataIn == 'X')
    {
      Ld += -0.01;
      Serial1.println(Ld);
    }
  }
  }
*/
////Test Motor RPM in relation to PWM.
void testMotors() {
  {
    //while (!Serial);
    int rpm, changeA, changeB = 0;


    //Serial.println(x);
    //    myEncA.write(0);
    myEncB.write(0);
    while (1) {
      move(1, 50, 1, 1);
      move(2, 50, 1, 1);
      delay(1000);
      move(1, 50, 0, 1);
      move(2, 50, 0, 1);
      delay(1000);
      move(1, 150, 1, 1);
      move(2, 150, 1, 1);
      delay(1000);
      move(1, 150, 0, 1);
      move(2, 150, 0, 1);
      delay(1000);

      move(1, 0, 0, 1);
      move(2, 0, 0, 1);
      delay(1000);
    }
    while (1) {
      //      changeA = myEncA.read();
      changeB = myEncB.read();
      Serial.print(changeA);
      Serial.print(", ");
      Serial.println(changeB);
      if (changeB > 1000) {
        move(B, 50, 0, 1);
      }
      if (changeA < -1000) {
        move(A, 50, 0, 1);

      }

      if (changeB < -1000) {
        move(B, 50, 1, 1);

      }
      if (changeA > 1000) {
        move(A, 50, 1, 1);

      }
    }
    delay(200);
    //rpm = ((change * 5 * 60) / 12) / 50;
    //Serial.println(rpm);

  }

}



void KalmanInit() {
  TWBR = ((F_CPU / 400000L) - 8) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  //double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

#endif

  kalmanX.setAngle(roll); // Set starting angle

  gyroXangle = roll;
  compAngleX = roll;

  timer = micros();
}

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}












