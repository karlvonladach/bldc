/// MKS ESP32 FOC Open loop speed control example; Test Library：SimpleFOC 2.1.1 ; Tested hardware：MKS ESP32 FOC V1.0
/// Enter "T+number" in the serial port to set the speed of the two motors.For example, to set the motor to rotate at a speed of 10rad/s, input "T10"
/// When the motor is powered on, it will rotate at 5rad/s by default
/// When using your own motor, do remember to modify the default number of pole pairs, the value in BLDCMotor(7).
/// The default power supply voltage of the program is 12V.
/// Please remember to modify the voltage_power_supply , voltage_limit variable values when using other voltages for power supply

// Tengelykapcsoló aktuátor meghajtása
// Szükség szerint módosítva a simple FOC "open_loop_velocity" sketch-ből
// Török Bence 2024


//****************************************
//  Using Simple FOC motor control library
//****************************************
#include <SimpleFOC.h>
#include <Preferences.h>

//****************************************
//  Define motor and driver hardware
//****************************************

//Define motor with pole pairs (14 poles = 7 pairs)
BLDCMotor motor = BLDCMotor(7);

//Define driver with ESP32 pins
BLDCDriver3PWM driver  = BLDCDriver3PWM(26,27,14,21, NOT_SET, NOT_SET);

//Define current sense with ESP32 pins
InlineCurrentSense current_sense0 = InlineCurrentSense(0.01, 50.0, 35, 34, NOT_SET);

//****************************************
//  Define commanded variables (accessed by commander)
//****************************************

Preferences preferences;

//Motor voltage [V]
float motor_voltage = 8;

//Motor speed [rad/s]
float run_speed = 50.0;

//Time [ms] to reach "run_speed"
float acc_time = 250.0;

//Time [ms] to run after acceleration
float run_time = 250.0;

//****************************************
//  Define serial commander and callbacks to access reference variables
//****************************************

Commander command = Commander(Serial);
void doVoltage(char* cmd)  { command.scalar(&motor_voltage, cmd); preferences.begin("my-app", false); preferences.putFloat("motorVoltage",motor_voltage); preferences.end(); }
void doRunSpeed(char* cmd) { command.scalar(&run_speed, cmd);     preferences.begin("my-app", false); preferences.putFloat("runSpeed", run_speed);        preferences.end(); }
void doAccTime(char* cmd)  { command.scalar(&acc_time, cmd);      preferences.begin("my-app", false); preferences.putFloat("accTime", acc_time);          preferences.end(); }
void doRunTime(char* cmd)  { command.scalar(&run_time, cmd);      preferences.begin("my-app", false); preferences.putFloat("runTime", run_time);          preferences.end(); }

//****************************************
//  Actuator application variables
//****************************************

//State var for the actuation process
enum {
  ST_OPENING,
  ST_OPEN,
  ST_CLOSING,
  ST_CLOSED
} MyState = ST_OPEN;

//Actual motor speed during acceleration and run
float speed = 0.0;

//Magnitude of the motor current
float current_magnitude;

//Timer variables
unsigned long MyTime, PrevTime, dTime, runtime;
unsigned long PrevMeasTime = 0;
unsigned long PrevStateTime = 0;

//Motor speed to reach
float RUN_SPEED = 50.0f;

//Acceleration time [us]
float ACC_TIME = 250000.0f;

//Run time [us]
unsigned long RUN_TIME = 250000;

// Pin state
int last_state = 1;
int new_state = 1;

//****************************************
//  Program init
//****************************************

void setup() {

  preferences.begin("my-app", false);

  //Get Motor voltage [V] from NVS
  motor_voltage = preferences.getFloat("motorVoltage", 8.0);

  //Get Motor speed [rad/s] from NVS
  run_speed = preferences.getFloat("runSpeed", 50.0);
  RUN_SPEED = constrain(run_speed, 5.0, 200.0);

  //Get Time [ms] to reach "run_speed" from NVS
  acc_time = preferences.getFloat("accTime", 250.0);
  ACC_TIME = constrain((acc_time * 1000.0), 50000.0, 500000.0);

  //Get Time [ms] to run after acceleration from NVS
  run_time = preferences.getFloat("runTime", 250.0);
  RUN_TIME = constrain((run_time * 1000.0), 50000.0, 500000.0);

  preferences.end();

  // Set board input voltage
  driver.voltage_power_supply = 12;

  // Initialize driver
  driver.init();

  // Link driver to motor
  motor.linkDriver(&driver); 

  // Set motor voltage // Don't care: updated in every cycle          
  motor.voltage_limit = motor_voltage;

  // Set motor KV
  // -not used-

  // Set motor resistance [Ohms]
  // -not used-

  // Set motor Current
  // -not used-

  // Set motor safe max speed [rad/s]
  motor.velocity_limit = 200;

  // Init current sense
  current_sense0.init();

  // Current sense polarity reversed!
  current_sense0.gain_b *= -1;
  current_sense0.gain_a *= -1;

  // Open Loop Control Mode Setting
  motor.controller = MotionControlType::velocity_openloop;

  // Init motor
  motor.init();

  // Add possible commands to commander
  command.add('V', doVoltage, "motor_voltage");
  command.add('W', doRunSpeed, "run_speed");
  command.add('A', doAccTime, "acc_time");
  command.add('R', doRunTime, "run_time");

  // Init serial communication
  Serial.begin(115200);
  Serial.println("Motor ready!");
  _delay(1000);

  // Init digital input (for control line from VESC)
  pinMode(19, INPUT_PULLDOWN);

  // Start with motor stopped
  motor.disable();

} //eof setup()

//****************************************
//  Cyclic progam
//  -cycle time not defined, nor fixed
//  -timing possible only via accessing system-time
//****************************************
void loop() {

  // Calculate elapsed time since last loop entry
  MyTime = micros();
  dTime = MyTime - PrevTime;
  PrevTime = MyTime;

  // Measure current every 50ms
  if ((MyTime - PrevMeasTime) > 50000){
    current_magnitude = current_sense0.getDCCurrent();
    if (current_magnitude > 0.001){
      Serial.print("\tI = ");
      Serial.println(current_magnitude*1000); // milli Amps
    }
    PrevMeasTime = MyTime;
  }

  // Read pin state every 6ms
  if ((MyTime - PrevStateTime) > 6000){
    last_state = new_state;
    new_state = digitalRead(19);
    PrevStateTime = MyTime;
  }

  // Manage states of actuating process
  switch(MyState)
  {
    case ST_CLOSING:
      if(last_state == 0 && new_state == 0)  //Check for open command
      {
        speed = 0.0;
        motor.enable();
        MyState = ST_OPENING;
        break;
      }

      if(speed < RUN_SPEED)   //Accelerating
      {
        speed += dTime * RUN_SPEED / ACC_TIME;
        runtime = 0;
      }
      else                    //Running
      {
        runtime += dTime;
      }

      if(runtime > RUN_TIME)  //Run time elapsed, actuator is closed
      {
        runtime = 0;
        motor.disable();
        MyState = ST_CLOSED;
      }
    break;

    case ST_CLOSED:           //Wait for open command
      if(last_state == 0 && new_state == 0)
      {
        speed = 0.0;
        motor.enable();
        MyState = ST_OPENING;
      }
    break;

    case ST_OPENING:
      if(last_state == 1 && new_state == 1)  //Check for close command
      {
        speed = 0.0;
        motor.enable();
        MyState = ST_CLOSING;
        break;
      }

      if(speed > (-RUN_SPEED))  //Accelerating
      {
        //speed += 0.001;
        speed -= dTime * RUN_SPEED / ACC_TIME;
        runtime = 0;
      }
      else                      //Running
      {
        runtime += dTime;
      }

      if(runtime > RUN_TIME)    //Run time elapsed, actuator is opened
      {
        runtime = 0;
        motor.disable();
        MyState = ST_OPEN;
      }
    break;

  case ST_OPEN:
      if(last_state == 1 && new_state == 1)  //Wait for close command
      {
        speed = 0.0;
        motor.enable();
        MyState = ST_CLOSING;
      }
    break;
  }

  // Give actual motor speed for simple FOC
  motor.move(speed);

  // Refresh motor voltage
  motor.voltage_limit = motor_voltage;

  // Refresh actuator speed
  RUN_SPEED = constrain(run_speed, 5.0, 200.0);

  // Refresh actuator acceleration time, change ms to us
  ACC_TIME = constrain((acc_time * 1000.0), 50000.0, 500000.0);

  // Refresh actuator run time, change ms to us
  RUN_TIME = constrain((run_time * 1000.0), 50000.0, 500000.0);

  // Update commander
  command.run();

} //eof loop()
