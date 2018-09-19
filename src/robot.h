#ifndef ROBOT_H
#define ROBOT_H

//!  ROBOT Class
/*!
  Class incorporating the robot. This class is used to define state machines, control algorithms, sensor readings,...
  It should be interfaced with the communicator to send data to the world.
*/

#include <inttypes.h>
#include "math.h"

#include <microOS.h>
#include <encoder_sensor.h>
#include <sharp41S.h>
#include <l293d.h>
#include "input.h"

class Robot
{
private:
	uint8_t _ID;			//give the robot an ID so that you can recognize it
	uint8_t _type;

	// Give the robot some sensors
	Sensor1D* _encoder1;
	Sensor1D* _encoder2;
	Sensor1D* _distance1;
	Sensor1D* _distance2;
	Sensor1D* _pendulum;

	// Give the robot some motors
	HBridgeInterface* _motor1;
	HBridgeInterface* _motor2;

	// Interface the buttons
	bool _button_states[8] = {false,false,false,false,false,false,false,false};
	bool toggleButton(uint8_t button);
	bool controlEnabled();

public:
    Robot(uint8_t ID = 0);

	////////
	/// FUNC
	void init();			//set up the robot
	void controllerHook();	//update function which can be executed continuously
	void resetEncoders();	//reset the encoders
	void resetPendulum();	//reset pendulum zero position


//user defined functions:
  void motorsUpdate(float motor1, float motor2);
  float getInputs(void);//read inputs from flash
  void controlScheme(void);
  void testEncoders(void);

//user defined variables:
  float u1[2] = {0.0, 0.0};     //output of controller 1
  float e1[2] = {0.0, 0.0};     //input of controller 1
  float y1[4] = {0,0,0,0};           //position 1
  float v1 = 0;             //speed 1
  float v1x = 0;

  float u2[2] = {0.0, 0.0};     //output of controller 2
  float e2[2] = {0.0, 0.0};     //input of controller 2
  float y2[4] = {0,0,0,0};           //position 2
  float v2 = 0;              //speed 2
  float v2x = 0;
  float integral = 0;
  
  //These variables are prepared for PID control of position
  float up[3] = {0.0, 0.0, 0.0};
  float ep[3] = {0.0, 0.0, 0.0};

  //States
  float x1 = 0.0;
  float x2 = 0.0;
  float x3 = 0.0;
  float p[3] = {0.0, 0.0, 0};
  float vpr[11] = {0,0,0,0,0,0,0,0,0,0,0};

  float xe1[2] = {0.0, 0.0};
  float xe2[2] = {0.0, 0.0};
  float xe3[2] = {0.0, 0.0};

  float y = 0.0;
  float nu1 = 0.0;
  float nu2 = 0.0;
  float nu3 = 0.0;
  float x_cart = 0.0;

///FIR
  float h[11] = {0.0145,0.0306,0.0725,0.1245,0.1665,0.1826,0.1665,0.1245,0.0725,0.0306,0.0145};
  
	///////
	/// GET
    uint8_t id();
	uint8_t type();

   // Event callbacks
   void button1callback();
   void button2callback();
   void button3callback();
   void button4callback();
   void button5callback();
   void button6callback();
   void button7callback();
   void button8callback();
};

#endif //ROBOT_H
