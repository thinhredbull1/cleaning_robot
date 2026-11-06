#include "config.h"
// #include <util/atomic.h>
// #include "digitalWriteFast.h"
double m_per_count_l = 0;
double m_per_count_r = 0;
double robot_width = 1;

// right ~ 85 count/ vong
// left ~ 720

volatile int speed_desired[] = {0, 0, 0, 0};
volatile int encoder_count[NMOTORS];
long publish_encoder[NMOTORS];
float last_encoder[NMOTORS];
unsigned long publish_time = 0;
volatile int pos[NMOTORS] = {0, 0, 0, 0};
SimplePID pid[NMOTORS];
int encod_state[NMOTORS];
robot_pos robotGlobalPos;
void control_motor(int motor, int speed)
{
  speed *= motor_sign[motor];

  bool direct = speed > 0 ? 1 : 0;
  speed = constrain(speed, -255, 255);

  if (speed == 0)
  {
    analogWrite(pwm[motor], 0);
    digitalWrite(dir[motor], 0);
    return;
  }
  // digitalWrite(dir[motor],direct);
  if (direct)
  {
    analogWrite(pwm[motor], 255 - abs(speed));
    digitalWrite(dir[motor], 1);
  }
  else
  {
    analogWrite(pwm[motor], abs(speed));
    digitalWrite(dir[motor], 0);
  }
}
void IRAM_ATTR encoderLeftMotor2()
{
  static bool old_a = false;
  bool newA = digitalRead(enca[M_L_DOWN]);
  bool newB = digitalRead(encb[M_L_DOWN]);
  int delta = 0;
  if (newA && !old_a)
  {
    delta = newB > 0 ? dir_encod[M_L_DOWN] : -dir_encod[M_L_DOWN];
  } // rising
  else
  {
    delta = newB > 0 ? -dir_encod[M_L_DOWN] : dir_encod[M_L_DOWN];
  }
  encoder_count[M_L_DOWN] -= delta;
  old_a = newA;
}
void IRAM_ATTR encoderRightMotor2()
{
  static bool old_a = false;
  bool newA = digitalRead(enca[M_R_DOWN]);
  bool newB = digitalRead(encb[M_R_DOWN]);
  int delta = 0;
  if (newA && !old_a)
  {
    delta = newB > 0 ? dir_encod[M_R_DOWN] : -dir_encod[M_R_DOWN];
  } // rising
  else
  {
    delta = newB > 0 ? -dir_encod[M_R_DOWN] : dir_encod[M_R_DOWN];
  }
  encoder_count[M_R_DOWN] -= delta;
  old_a = newA;
}
void IRAM_ATTR encoderLeftMotor()
{
  static bool old_a = false;
  bool newA = digitalRead(enca[M_L_UP]);
  bool newB = digitalRead(encb[M_L_UP]);
  int delta = 0;

  if (newA && !old_a)
  {
    delta = newB > 0 ? dir_encod[M_L_UP] : -dir_encod[M_L_UP];
  } // rising
  else
  {
    delta = newB > 0 ? -dir_encod[M_L_UP] : dir_encod[M_L_UP];
  }
  encoder_count[M_L_UP] -= delta;
  old_a = newA;
}
void IRAM_ATTR encoderRightMotor()
{
  static bool old_a = false;
  bool newA = digitalRead(enca[M_R_UP]);
  bool newB = digitalRead(encb[M_R_UP]);
  int delta = 0;
  if (newA && !old_a)
  {
    delta = newB > 0 ? dir_encod[M_R_UP] : -dir_encod[M_R_UP];
  } // rising
  else
  {
    delta = newB > 0 ? -dir_encod[M_R_UP] : dir_encod[M_R_UP];
  }
  encoder_count[M_R_UP] -= delta;
  old_a = newA;
}
// 15/15*15&15; --> speed_desired()
// 10:0.5#0; --> PID CONFIG
void receive_uart()
{
    if (Serial.available())
    {
        String c = Serial.readStringUntil(';');

        int index_now = c.indexOf("/");

        if (index_now != -1)
        {
            int motor = c.substring(0, index_now).toInt();
            int speed = c.substring(index_now + 1).toInt();

            if (motor >= 0 && motor < NMOTORS)
            {
                control_motor(motor, speed);
            }
        }
    }
}


void send_odom()
{
   for (int i = 0; i < NMOTORS; i++)
    {
        publish_encoder[i] = encoder_count[i];
    }
  String odom_data = String(publish_encoder[M_L_UP]) + "/" + String(publish_encoder[M_L_DOWN]) + "&" + String(publish_encoder[M_R_UP]) + "*" + String(publish_encoder[M_R_DOWN]) + ";";
  Serial.println(odom_data);
  // for(int i=0;i<NMOTORS;i++){
  //   publish_encoder[i]=0;
  // }
}
void setup()
{
  Serial.begin(57600);
  for (int k = 0; k < NMOTORS; k++)
  {
    pinMode(enca[k], INPUT_PULLUP);
    pinMode(encb[k], INPUT_PULLUP);
    pinMode(pwm[k], OUTPUT);
    pinMode(dir[k], OUTPUT);
    pid[k].setParams(p_gain_default, i_gain_default, d_gain_default, 255);
    // pinMode(dir2[k], OUTPUT);
  }
  for (int i = 0; i < 4; i++)
  {
    control_motor(i, 0);
  }


  attachInterrupt(digitalPinToInterrupt(enca[M_L_UP]), encoderLeftMotor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[M_R_UP]), encoderRightMotor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[M_L_DOWN]), encoderLeftMotor2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[M_R_DOWN]), encoderRightMotor2, CHANGE);
  publish_time = micros();
}
void loop()
{
  static unsigned long time_loop_pid = millis();
  static unsigned long time_publish = millis();
  static int last_a_left = 0;
  static int last_a_right = 0;
  static int last_b_left = 0;
  static int last_b_right = 0;
  int delta_now = 0;
  // control_motor(0,-5u0);
    receive_uart();

  callFunctionPeriodically(send_odom, LOOP_PUB, time_publish);     // gui len pi neu dung ros serial
}