//#include <PWMServo.h>
#include <Wire.h>
//#include <WSWire.h>
#include <TimerOne.h>                                 // Header file for TimerOne library

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AutoPID.h>
#include <pidautotuner.h>

//PID constants
double kp = 0;
double ki = 0;
double kd = 0;
unsigned long previousTimePID;
const int PID_limit_low = -255;
const int PID_limit_high = 255;
double Setpoint, Input, Output;
AutoPID PID_obj(&Input, &Setpoint, &Output, PID_limit_low, PID_limit_high, kp, ki, kd);

#include <Adafruit_PWMServoDriver.h>
#include <RotaryEncoder.h>
#include <Pushbutton.h>

#include <EEPROMex.h>

#define BUTTON_PIN A1
RotaryEncoder encoder(A2, A3);
Pushbutton button(BUTTON_PIN);

#define _DEBUG

#define LED_BLUE 0
#define LED_GREEN 1
#define LED_RED 2
#define LED_STRIP_RIGHT 4
#define LED_STRIP_CENTER 5
#define LED_STRIP_LEFT 6

#define LEFT 0
#define CENTER 1
#define RIGHT 2

static int pos = 0;   // RotaryEncoder Pos

//Prototypes
void Blue_Serial_Send(byte mode, float data_1 = 0, float data_2 = 0, float data_3 = 0, float data_4 = 0);
void Led_strip_indication(unsigned char _led, unsigned char _power = 10);
// Prototypes end

float ema_ema = 0;
float ema = 0;
float alpha = 0.2;

float ema_ema_US = 0;
float ema_US = 0;
float alpha_US = 0.2;

#define b_rate 115200
#define d_time 60

#define ONBOARD_LED_PIN 13

#define PIN_MotorR_A  9     // 7
#define PIN_MotorR_B  4     // 8
#define PIN_MotorL_A  7     // 9
#define PIN_MotorL_B  8     // 4
#define PIN_pwm_1     5
#define PIN_pwm_2     6

#define MODE_NOTHING 0
#define MODE_PID    1
#define MODE_WHEEL  2
#define MODE_TEST   3
#define MODE_NOPID    4
#define MODE_PID_CALIBRATION    5
#define MODE_TURN1 6
#define MODE_TURN2 7
#define MODE_SERVO_UD_SET 8
#define MODE_GO_STRAIT 9

#define MODE_ONE_ANGLE    1
#define MODE_ALL_ANGLES   2
#define MODE_ANGLE_SETPOINT    3

#define cruse_time 10   // sec

double angle_1 = 0;
double angle_1_filtered = 0;
boolean send_angle = false;
double angle_to_turn = 999;

boolean HB = false;

byte mode = MODE_NOTHING;
byte encoder_mode = LEFT;

//byte MAX_HIGH = 60; //of #define
const byte MAX_HIGH = 100; //for 5v
const byte MIN_MOTOR_1 = 0;   //was 48 old motor
const byte MIN_MOTOR_2 = 0;   //was 60 old motor

int wheel_x = 0;
int wheel_y = 0;

//int trigState = LOW; //state of trigPin
int intervalUS = 50; // interval in milliseconds at which trigPin turns on
unsigned long pMsUS = 0;

unsigned long previousMillis = 0;
const unsigned long interval = 600;
unsigned long currentMillis;

unsigned long pM = 0;
const unsigned long interval2 = 40;      // Main control interval

unsigned long pM_3 = 0;
const unsigned long interval3 = 100;

unsigned long pM_1_sec = 0;
const unsigned long interval_1_sec = 1000;

unsigned long pM_servo = 0;
const unsigned long interval_servo = 35;

unsigned long pM_run = 0;
unsigned long interval_run = 5000;

unsigned long pMsStartALL = 0;
unsigned long intervalStartALL = 60 * 4 * 1000;

unsigned long pMsStartSTRAIGHT = 0;
unsigned long intervalSTRAIGHT = 1 * 1000;

boolean pot_change = false;
boolean debug_sent_bt_data = false;

boolean motors_runing = false;
byte motorR_max_speed = MAX_HIGH * 0.87;   //* 0.67; //* 0.87;
byte motorL_max_speed = MAX_HIGH * 1;   //* 0.80; //* 1;
bool speed_reduced(false);


int motor1_speed_out;
int motor2_speed_out;
int constrainedOutput_motor1;
int constrainedOutput_motor2;
int boat_speed = 150;
int boat_turn = 0;
int far_circle_angle = 0;

int left_right_move = 0;
int forward_back_move = 0;;

const int V_scalar = 0.333 * 1000;     // v=0.333 m/s or v=333 m/ms
unsigned long sea_distance_calc = 0;
unsigned long time_started;

#define trigPin 2 //set US pins
#define echoPin 3
volatile byte US_flag = 0;
volatile unsigned long US_timers[2] = {0, 0};
unsigned long US_echo = 100;
float US_alpha = 0.1, US_distance = -1;
#define US_arr_size 10
float US_distance_arr[US_arr_size];
int US_distance_cm;

int turn_val = 255;

#define FILTER_SIZE 20
#define DATA_INIT -99999
double data_in[FILTER_SIZE] = {0};
boolean data_bool[FILTER_SIZE] = {0};
const double MAX_VAR_ALLOWED = 2.0;

#define TX 11
#define RX 12
#define BT_Baud 115200
SoftwareSerial BlueSerial(TX, RX);  //Bluetooth TX to 11 and Bluetooth RX to 12.
SoftwareSerial mpuSerial(2, 3);  // ArduMPU is (7,8)

boolean g_command_ready(false);
String g_command;
boolean g_command_ready_s(false);
String g_command_s;
boolean i2c_command_ready_s(false);
String i2c_command_str;
boolean g_command_ready_mpu(false);
String g_command_mpu;

unsigned int count = 0;
unsigned int var_pot;
unsigned int old_var_pot;

int turn_step = 0;

double mpu_temperature = 0;

Adafruit_PWMServoDriver PWM_module = Adafruit_PWMServoDriver();
#define SERVO_UP_DOWN_MIN  135 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_LR_CENTER  325 // this is the 'minimum' pulse length count (out of 4096)

#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define SERVO_UP_DOWN 14
#define SERVO_LEFT_RIGHT 15
#define S_U 1
#define S_D 2
#define S_L 3
#define S_R 4
#define S_LR_OFF 5
#define S_UD_OFF 6
#define pulseLenCW 485
#define pulseLenCCW 515

#define TIMER_US 50                                   // 50 uS timer duration 
#define TICK_COUNTS 4000                              // 200 mS worth of timer ticks

volatile long echo_start = 0;                         // Records start of echo pulse
volatile long echo_end = 0;                           // Records end of echo pulse
volatile long echo_duration = 0;                      // Duration - difference between end and start
volatile int trigger_time_count = 0;                  // Count down counter to trigger pulse time
volatile long range_flasher_counter = 0;              // Count down counter for flashing distance LED
const unsigned char servo_prog_new[] PROGMEM = {};

int servo_prog_step = -1;
boolean RUN_started(false);
boolean RUN_need_to_stop(false);
boolean prog_with_servo(false);
bool led_state = true;

const int servo_up_pos = SERVO_UP_DOWN_MIN;
const int servo_left_pos = SERVO_LR_CENTER;
int servo_counter = 0;
int servo_step = -1;

int FULL_PROG_STEP = -1;

void setup() {

  SetInOut_pins();
  Servo_Initialize();
  Timer1.initialize(TIMER_US);                        // Initialise timer 1
  Timer1.attachInterrupt( timerIsr );                 // Attach interrupt to the timer service routine

  Serial.begin(b_rate);
  BlueSerial.begin(BT_Baud);
  //  mpuSerial.begin(115200);

  EEPROM_Read_PID();
  kp = 20; ki = 0; kd = 5;
  //  kp: 23.14 ki: 0.50 kd: 715.64
  //  kp = 23.14; ki = 0.50 kd = 715.64;

  //if angle is more than 60 degrees below or above setpoint, OUTPUT will be set to min or max respectively
  PID_obj.setBangBang(60);
  PID_obj.setGains(kp, ki, kd);

  Serial.println(F("Setpoint=0"));
  Setpoint = 0.0;

  //  EEPROM_Write_double(0, 0, 0);
  //  Init_data_in_arr();

  PWM_Led_off();
  Led_strip_off();
  US_dist_arr_INIT();

  Serial.println(F("---Starting loop---"));
  //  randomSeed(analogRead(0));  //for noise gen. - DEBUG
}

void loop() {
  currentMillis = millis();

  Led_strip_indication(encoder_mode);
  encoder_func();
  US();
  mpu_Serial();           //MPU update
  Blue_Serial_Rec();

  if (currentMillis - pM >= interval2) {   //Delay2   -   Main code delay - 40ms
    pM = currentMillis;

    if (FULL_PROG_STEP != -1) DO_FULL_PROGRAM2();

    // angle_1 is updating automaticly
    Input = angle_1_filtered;               //= angle_1;

    if (mode == MODE_PID) {
      PID_obj.run();
      motors_wheel(Output, 255); //takes "Output" and sets motors speed
    }

    if (mode == MODE_WHEEL) {
      left_right_move = map(wheel_x, 0, 200, -255, 255);
      forward_back_move = map(wheel_y, 0, 380, 255, -255);
      motors_wheel(left_right_move, forward_back_move);
    }

    if (mode == MODE_TEST) {
    }

    if (mode == MODE_TURN1) {
      turn_val = boat_local_turn(boat_turn, Input);
      motors_spin(turn_val, boat_speed);
    }

    if (mode == MODE_TURN2) {
      motors_wheel(-90, 200);
    }

    if (mode == MODE_NOTHING) {
      motors_runing = false;
      angle_to_turn = 999;
    }

    motors();

    Serial_Print_data(motor1_speed_out, motor2_speed_out, 0); //distance);

  } // end (currentMillis - pM >= interval2)

  if (currentMillis - pM_3 >= interval3) {   //Delay3   -   BT send code delay
    pM_3 = currentMillis;
    // send angle to Android
    if (send_angle == true) {
      //      Blue_Serial_Send(MODE_ONE_ANGLE, Input);  //Input=angle_1_filtered
      Blue_Serial_Send(MODE_ANGLE_SETPOINT, Input, Setpoint);

    }

  } // end (currentMillis - pM_3 >= interval3)

  if (currentMillis - pM_servo >= interval_servo) {
    pM_servo = currentMillis;
    Servo_move();
  }

  if (currentMillis - pMsStartALL >=   intervalStartALL) {
    FULL_PROG_STEP = -1;
    motors_off();
    mode = MODE_NOTHING;
  }

} //loop end
