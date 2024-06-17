// Automation V3
#include <Servo.h>
#include <math.h>

#define BUTTON 4
#define SAVED_POINTS_NUMBER  6

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

typedef struct P {
  float x, y;
  int yawAngle, neckAngle, clawAngle;
} Point;

Point point;

Point pointArr[SAVED_POINTS_NUMBER];
byte i = 0;
bool mask = 1, save = 0;

int prevVal1 = 90, prevX = 90, prevY = 90, prevVal4 = 90, prevVal5 = 90;

int val1;
float val2X;
float val3Y;
int val4 = 90;
int val5;


void setup() {
  Serial.begin(115200);

  servo1.attach(9);
  servo2.attach(6);
  servo3.attach(5);
  servo4.attach(3);
  servo5.attach(11);

  pinMode(BUTTON, INPUT);
}

void loop() {

  Serial.println("Enter point x, y, yaw, claw");
  while (digitalRead(BUTTON)) {
      // using my mapd function made the robot move very smooth
    val1 = map(analogRead(0), 0, 1023, 0, 180);
    val2X = mapf(analogRead(1), 0, 1023, 0, 15);      // x-axix points
    val3Y = mapf(analogRead(2), 0, 1023, -10, 13);  // y-axis points
    val4 = map(analogRead(3), 0, 1023, 0, 180);
    val5 = map(analogRead(6), 0, 1023, 90, 180);

    servo1.write(val1);
    gotoPointIn2DPlane(val2X ,val3Y);
    servo4.write(val4);
    servo5.write(val5);
  }
  if (!digitalRead(BUTTON)) {
    save = 1;
    point.x = val2X;
    Serial.println(point.x);
    point.y = val3Y;
    Serial.println(point.y);
    point.yawAngle = val1;
    Serial.println(val1);
    point.clawAngle = val5;
    Serial.println(val5);
    Serial.println("____Points_Saved____");
    mask = 0;
  } 
  else if (digitalRead(BUTTON)) {
    mask = 1;
  }
  delay(1000);

  //to save the 6 points
  if(save){
    if(i >= SAVED_POINTS_NUMBER){
      Serial.println("Can't stote more points");
      return;
    } 
    pointArr[i] = {.x = point.x, .y = point.y, .yawAngle = val1, .neckAngle = val4, .clawAngle = val5};
    Serial.println("__New Point SAVED__");
    i++;
    save = 0;
    Serial.println("Save mode DeActivated");
  }

  // infinite loop to repeat the saved positions of the two joints
  while(i >= SAVED_POINTS_NUMBER){
    delay(1000);
    for(int j = i-1; j > 0; j--){
      servo_smooth(&servo1 ,pointArr[j].yawAngle);
      gotoPointIn2DPlane(pointArr[j].x, pointArr[j].y);
      servo_smooth(&servo4 ,pointArr[j].neckAngle);
      servo_smooth(&servo5 ,pointArr[j].clawAngle);
      Serial.print("Excuting point number: ");
      Serial.println(j+1);
      delay(800);
    }
    for(int j = 0; j < i; j++){
      servo_smooth(&servo1 ,pointArr[j].yawAngle);
      gotoPointIn2DPlane(pointArr[j].x, pointArr[j].y);
      servo_smooth(&servo4 ,pointArr[j].neckAngle);
      servo_smooth(&servo5 ,pointArr[j].clawAngle);
      Serial.print("Excuting point number: ");
      Serial.println(j+1);
      delay(800);
    }
  }

}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double mapd(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// send the refference of X_Angle ,Y_Angle variables ,point x ,and point y.
void calculateInverseKinematics2Joints(float *X_AnglePtr, float *Y_AnglePtr, float pX, float pY) {
  ////////////////////////////////////////////////////////////////////////////////////
  //                         Inverse Kinematics Claculations
  float alfa, xMinusAlfa, beta, theta, hyp, shortLeg = 6.5, longLeg = 13.5;
  int offsetValueForServo3 = 180;

  hyp = hypot(pX, pY);

  alfa = acos(((hyp * hyp) + (shortLeg * shortLeg) - (longLeg * longLeg)) / (2 * hyp * shortLeg));
  alfa = (alfa / PI) * 180;

  xMinusAlfa = atan(pY / pX);
  xMinusAlfa = (xMinusAlfa / PI) * 180;
  *X_AnglePtr = (xMinusAlfa + alfa);

  beta = acos(((longLeg * longLeg) + (shortLeg * shortLeg) - (hyp * hyp)) / (2 * longLeg * shortLeg));
  beta = (beta / PI) * 180;
  *Y_AnglePtr = (offsetValueForServo3 - beta);
  ////////////////////////////////////////////////////////////////////////////////////
}

void gotoPointIn2DPlane(float x, float y) {

  float Y_Angle, X_Angle;
  calculateInverseKinematics2Joints(&X_Angle, &Y_Angle, x, y);
  servo_smooth_2Servos(&servo2, &servo3, X_Angle, Y_Angle);
}

void servo_smooth(Servo *servo, int val) {

  float smoothedval, prevval = (*servo).read();
  while (int(smoothedval) != val) {
    if (val == int(smoothedval) + 1) {
      smoothedval = val;
      break;
    }
    // *** smoothing ***
    smoothedval = (val * 0.05) + (prevval * 0.95);
    prevval = smoothedval;

    // *** Move the servo to the desired position
    (*servo).write(int(smoothedval));
    // Serial.println(int(smoothedval));
    delay(10);
  }
}
void servo_smooth_2Servos(Servo *servoOne, Servo *servoTwo, int valOne, int valTwo) { // my brand new function :)))))

  float smoothedvalOne, prevvalOne = (*servoOne).read();
  float smoothedvalTwo, prevvalTwo = (*servoTwo).read();
  const int IntPrevvalOne = prevvalOne, IntPrevvalTwo = prevvalTwo;

  while (int(smoothedvalOne) != valOne || int(smoothedvalTwo) != valTwo) {
    if (valOne == int(smoothedvalOne) + 1) {
      smoothedvalOne = valOne;
      break;
    }

    if (valTwo == int(smoothedvalTwo) + 1) {
      smoothedvalTwo = valTwo;
      break;
    }

    // *** smoothing servo1 ***
    smoothedvalOne = (valOne * 0.05) + (prevvalOne * 0.95);
    prevvalOne = smoothedvalOne;

    // *** smoothing servo2 ***
    smoothedvalTwo = (valTwo * 0.05) + (prevvalTwo * 0.95);
    prevvalTwo = smoothedvalTwo;

    // *** Move the servo1 to the desired position
    (*servoOne).write(int(smoothedvalOne));

    // *** Move the servo2 to the desired position
    (*servoTwo).write(int(smoothedvalTwo));

    delay(10);
  }
}