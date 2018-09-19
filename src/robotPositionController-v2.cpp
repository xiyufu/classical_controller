#include "robot.h"

#define ENC1_PINA		18
#define ENC1_PINB		32
#define ENC2_PINA		19
#define ENC2_PINB		42
#define DIST1_PIN		A0
#define DIST2_PIN		A1
#define MOT1_PIN_IN1	8u
#define MOT1_PIN_IN2	12u
#define MOT1_PIN_EN		6u
#define MOT2_PIN_IN1	7u
#define MOT2_PIN_IN2	4u
#define MOT2_PIN_EN		5u
#define PENDULUM_PIN	A3
#define LED1_PIN 		34
#define LED2_PIN 		40
#define BATTERY_VOLTAGE	6000


Robot::Robot(uint8_t ID):
  _ID(ID),
  _type(20),
  _encoder1(new EncoderSensor(ENC1_PINA, ENC1_PINB)),
  _encoder2(new EncoderSensor(ENC2_PINA, ENC2_PINB)),
  _distance1(new Sharp41S(DIST1_PIN)),
  _distance2(new Sharp41S(DIST2_PIN)),
  _pendulum(new AnalogSensor(PENDULUM_PIN, 12)),
  _motor1(new L293D(MOT1_PIN_IN1, MOT1_PIN_IN2, MOT1_PIN_EN, BATTERY_VOLTAGE)),
  _motor2(new L293D(MOT2_PIN_IN1, MOT2_PIN_IN2, MOT2_PIN_EN, BATTERY_VOLTAGE))
{
  _pendulum->setScale(2.0f * M_PI / 1023.0f);
}

void Robot::init() {
  //initialize the robot - sort of starting procedure
  resetEncoders();
}

void Robot::controllerHook() {
  //do something that is periodic: reading from sensors, setting the motors, updating variables sent to the pc..

  if (controlEnabled()) {
    //write the control in here
    controlScheme();
  } else {
    //set motor voltage to zero or it will keep on running...
    motorsUpdate(0, 0);
  }
}

void Robot::motorsUpdate(float motor1, float motor2) {
  _motor1->setBridgeVoltage (motor1);
  _motor2->setBridgeVoltage (motor2);
}

float Robot::getInputs(void) {
  //Using flash to store large array, see input.h to check data.
  static int k = 0;
  float u = 0.0;
  int N = 1000;
  if (k < N) {
    u = pgm_read_float_near(input3 + k);
    k++;
  }

  return u;
}

void Robot::controlScheme(void) {

  static int ts = 0;
  
////read reference////
  float r = System.getGPinFloat(0); // [rad]
////read the sensors////
  int encValue1 = _encoder1->readRawValue (); // [0.004 rad]
  int encValue2 = _encoder2->readRawValue (); // [0.004 rad]
  
//////////////////////Proportional controller////////////////////////

////get Kp from the computer////
  float Kp = System.getGPinFloat(1);
////calculate the position////
  float p1 = -1 * encValue1*0.004; // [rad]
  float p2 = encValue2*0.004;      // [rad]
////calculate the position error////
  float ep1 = r - p1;
  float ep2 = r - p2;
////calculate the reference speed of PI controller////
  float rv1 = Kp * ep1;
  float rv2 = Kp * ep2;
//////////////////////////////////////////////////////////////////////

/////////////////////PI controller for motor 2 position loop////////////////////////

  float Ki = System.getGPinFloat(2);
  if(ep2<0.1&&ep2>-0.1){ //to prevent integral windup, only start the integral part when ep is small.     
    integral += ep2;
    rv2 += Ki*integral;
  }

////////////////////////////////////////////////////////////////////////////////////

////////////////////////////PI controller of speed loop///////////////////////////////

////motor2////
  y2[0] = y2[1];
  y2[1] = encValue2;  //position, unit: 0.004 rad
  v2 = (y2[1] - y2[0]) * 0.004 / 0.01; //speed, unit: rad/s
  e2[0] = e2[1];
  e2[1] = rv2 - v2; //error, unit: rad/s
  u2[0] = u2[1];
  u2[1] = u2[0] + 1245 * e2[1] - 996 * e2[0]; // output of controller, unit: mV.
//  if (ep2 <= 0.01&&ep2>=-0.01){
//    ts++;
//    if (ts>5){
//      u2[1] = 0;
//      u2[0] = 0;
//    }
//  }
////motor1////
  y1[0] = y1[1];
  y1[1] = -1 * encValue1; //position, unit: 0.004 rad
  v1 = (y1[1] - y1[0]) * 0.004 / 0.01; //speed, unit: rad/s
  e1[0] = e1[1];
  e1[1] = rv1 - v1; //error, unit: rad/s
  u1[0] = u1[1];
  u1[1] = u1[0] + 1245 * e1[1] - 996 * e1[0];// output of controller, unit: mV.
  if (e1[1] == 0 && e1[0] == 0 && ep1 == 0){
    //Once we reached the reference position, e = 0. u will not change. We must set it to 0 manually.
    ts++;
    if (ts>10)
      u1[1] = 0;
  }
////saturation//// 
//This is very important
  if (u1[1] > 6000)
    u1[1] = 6000;
  if (u1[1] < -6000)
    u1[1] = -6000;
  if (u2[1] > 6000)
    u2[1] = 6000;
  if (u2[1] < -6000)
    u2[1] = -6000;
////set motors////
  motorsUpdate(u1[0], u2[1]);
///////////////////////////////////////////////////////////////////////

//////////////////////Observe the results///////////////////////
  System.setGPoutFloat(0, v2);    //speed
  System.setGPoutFloat(1, ep2); //error 
  System.setGPoutFloat(2, u2[1]); //control signal 
  System.setGPoutFloat(3, p2);    //position
  System.setGPoutFloat(4, integral);
////////////////////////////////////////////////////////////////
}

void Robot::testEncoders() {
  int encValue2 = _encoder2->readRawValue();

  System.setGPoutInt(0, encValue2);
}

void Robot::resetEncoders()
{
  _encoder1->init();
  _encoder2->init();
}

void Robot::resetPendulum()
{
  _pendulum->setOffset(_pendulum->readRawValue());
}

uint8_t Robot::id()
{
  return _ID;
}

uint8_t Robot::type()
{
  return _type;
}

bool Robot::toggleButton(uint8_t button)
{
  _button_states[button] = !_button_states[button];
  return _button_states[button];
}

bool Robot::controlEnabled()
{
  return _button_states[0];
}

void Robot::button1callback()
{
  if (toggleButton(0)) {
    System.println("Controller enabled.");
  } else {
    System.println("Controller disabled.");
  }
}

void Robot::button2callback()
{
  toggleButton(1);

  resetEncoders();
  resetPendulum();

  //reset the controller states
  u1[0] = u1[1] = u2[0] = u2[1] = 0;
  y1[0] = y1[1] = y2[0] = y2[1] = 0;
  e1[0] = e1[1] = e2[0] = e2[1] = 0;

  integral = 0;
  
  System.println("Reset.");
}
void Robot::button3callback()
{
  toggleButton(2);
}

void Robot::button4callback()
{
  toggleButton(3);
}

void Robot::button5callback()
{
  toggleButton(4);
}

void Robot::button6callback()
{
  toggleButton(5);
}

void Robot::button7callback()
{
  toggleButton(6);
}

void Robot::button8callback()
{
  toggleButton(7);
}
