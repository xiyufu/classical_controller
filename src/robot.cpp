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
 //    testEncoders();

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

  float rv = 0.0;  
  float K1 = System.getGPinFloat(0);
  float K2 = System.getGPinFloat(1);
  float K3 = System.getGPinFloat(2);

  static float distance = _distance1->readCalibratedValue();
////////////////////////////////Measurements/////////////////////////////////////

////read the sensors
  int encValue1 = _encoder1->readRawValue (); // [0.004 rad]
  int encValue2 = _encoder2->readRawValue (); // [0.004 rad]
  float encP = _pendulum->readRawValue();
  int i = 0;
  float ve = 0;
////pendulum////
  p[0] = p[1]; // position of pendulum [rad]
  p[1] = (encP-367.0)*6.28/1024.0; // unit rad
  ve = (p[1]-p[0])/0.01; //velocity of pendulum [rad/s]
////motor1////
  y1[0] = y1[1];
  y1[1] = -1 * encValue1; //position of the motor1, unit: 0.004 rad
  v1 = (y1[1] - y1[0]) * 0.004 / 0.01; //velocity of the motor1, unit: rad/s
  v1x = 0.0325*v1;// velocity of the cart [m/s]
////motor2////
  y2[0] = y2[1];
  y2[1] = encValue2;  //position, unit: 0.004 rad
  v2 = (y2[1] - y2[0]) * 0.004 / 0.01; //speed, unit: rad/s
  v2x = 0.0325*v2;// m/s
////cart////
  x_cart = (v1x + v2x)/2;
////states////  
  x1 = p[1]; // measured state 1, the position of pendulum, phi [rad]
  x2 = ve - 7.41*x_cart; // measured state 2, phi_dot - x_dot/L [rad/s]
//  x3 = _distance1->readCalibratedValue() - distance; // measured state 3, position of the cart, x [m]
  
////////////////////////////////Measurements ends/////////////////////////////////////

////////////////////////////Estimator//////////////////////////////

  xe1[0] = xe1[1];
  xe2[0] = xe2[1];
//  xe3[0] = xe3[1];
  //prediction, x[n] = Ad*x[n-1]+Bd*u[n-1]
  xe1[1] = 1.0034*xe1[0] + 0.01*xe2[0];   
  xe2[1] = 0.6847*xe1[0] + 1.0034*xe2[0]; 
//  xe3[1] = xe3[0] - 0.01*x_cart;
  //update x[n] = x[n] + L*(y-Cd*x[n-1])
  nu1 = x1 - xe1[0];
  nu2 = x2 - xe2[0];
//  nu3 = x3 - xe3[0];
  xe1[1] = xe1[1] + 0.0895*nu1 + 0.01*nu2;
  xe2[1] = xe2[1] + 0.6847*nu1 + 0.0922*nu2;
//  xe3[1] = xe3[1] + 0.0833*nu3;

//  xe1[1] = x1;
//  xe2[1] = x2;
//  xe3[1] = x3;
////////////////////////////////////////////////////////////////////
  
////////////////////////////Regulator//////////////////////////////

//u[n] = -K*x[n]
  rv = -1*(K1*xe1[1] + K2*xe2[1] + K3*xe3[1]); //[m/s]
  
///////////////////////////////////////////////////////////////////


////////////////////////////////////////////PI controller////////////////////////////////////////
//The gain is 6 times larger than we we used in assignment 1. The codes at the end of this controller
//limit the voltage to [-6000, 6000]. Such a limitation equals to a smaller gain when saturation 
//happens, which keeps the motor stable.

////change rv into rad/s
  rv = rv/0.0325;
////motor2////
  e2[0] = e2[1];
  e2[1] = (rv - v2); //error, unit: rad/s  
  u2[0] = u2[1];
  u2[1] = u2[0] + 6000 * e2[1] - 2400 * e2[0]; // output of controller, unit: mV.
////motor1////
  e1[0] = e1[1];
  e1[1] = (rv - v1); //error, unit: rad/s
  u1[0] = u1[1];
  u1[1] = u1[0] + 6000 * e1[1] - 2400 * e1[0];// output of controller, unit: mV.
////motor saturation////  
  //This is very important for a faster tracking
  if (u1[1] > 6000)
    u1[1] = 6000;
  if (u1[1] < -6000)
    u1[1] = -6000;
  if (u2[1] > 6000)
    u2[1] = 6000;
  if (u2[1] < -6000)
    u2[1] = -6000;
////apply the voltage to motor////
  motorsUpdate(u1[1], u2[1]);
/////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////Observe the results///////////////////////
  System.setGPoutFloat(0, v1);  //observe speed1
  System.setGPoutFloat(1, xe1[1]);  //observe state1
  System.setGPoutFloat(2, xe2[1]);  //observe state2
  System.setGPoutFloat(3, xe3[1]);  //observe state3
  System.setGPoutFloat(4, ve);  //observe velocity of pendulum
  System.setGPoutFloat(5, x_cart); //observe cart speed
  System.setGPoutFloat(6, p[1]); //the pendulum position
//////////////////////////////////////////////////////////////////

}

void Robot::testEncoders() {
  float encP = _pendulum->readRawValue();
  p[0] = p[1];
  p[1] = encP;

  vpr[0] = (p[1] -p[0])*6.28/1024.0/0.01;

  System.setGPoutFloat(0, encP);
  System.setGPoutFloat(1, vpr[0]);
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

  x1 = 0;
  x2 = 0;
  x3 = 0;
  p[0] = p[1] = 0;

  xe1[1] = 0;
  xe2[1] = 0;
  xe3[1] = 0;

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
