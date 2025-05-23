#include "config.h"
// #include <util/atomic.h>
// #include "digitalWriteFast.h"
#include <ESP32Servo.h>
double m_per_count_l=0;
double m_per_count_r=0; 
double robot_width=1;
Servo SV1;
// right ~ 85 count/ vong
// left ~ 720
Servo ESC1;
volatile int speed_desired[] = { 0, 0 };
volatile int encoder_count[NMOTORS];
long publish_encoder[2];
float last_encoder[2];
unsigned long publish_time = 0;
volatile int pos[2] = { 0, 0 };
SimplePID pid[NMOTORS];
int encod_state[4];
robot_pos robotGlobalPos;
void control_motor(int motor, int speed) {
  bool direct = speed > 0 ? 1 : 0;
  if (speed == 0) {
    analogWrite(pwm[motor], 0);
    digitalWrite(dir[motor], 0);
    digitalWrite(dir2[motor], 0);
    return;
  }
  // digitalWrite(dir[motor],direct);
  if (direct) {
    analogWrite(pwm[motor], abs(speed));
    digitalWrite(dir[motor], 1);
    digitalWrite(dir2[motor], 0);
  } else {
    analogWrite(pwm[motor], abs(speed));
    digitalWrite(dir[motor], 0);
    digitalWrite(dir2[motor], 1);
  }
}
void IRAM_ATTR encoderLeftMotor() {
  static bool old_a = false;
  bool newA = digitalRead(enca[LEFT]);
  bool newB = digitalRead(encb[LEFT]);
  int delta = 0;
  if (newA && !old_a) {
    delta = newB > 0 ? dir_encod_M1 : -dir_encod_M1;
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod_M1 : dir_encod_M1;
  }
  encoder_count[LEFT] -= delta;
  old_a = newA;
}
void IRAM_ATTR encoderRightMotor() {
  static bool old_a = false;
  bool newA = digitalRead(enca[RIGHT]);
  bool newB = digitalRead(encb[RIGHT]);
  int delta = 0;
  if (newA && !old_a) {
    delta = newB > 0 ? dir_encod_M0 : -dir_encod_M0;
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod_M0 : dir_encod_M0;
  }
  encoder_count[RIGHT] -= delta;
  old_a = newA;
}
// 15/15; --> speed_desired()
// RESET; = reset odom
// 0,1,2,3I0,1; --> IO on off
// 1L2R3W; --> config odom param
// 10:0.5#0; --> PID CONFIG
bool receive_uart() {
  if (Serial.available()) {
    String c = Serial.readStringUntil(';');
    int index_now = c.indexOf("/");
    int index_kp_desired = c.indexOf(":");
    int config_param=c.indexOf("L");
    int reset_index=c.indexOf("RESET");
    int IO_index=c.indexOf("I");
    if (index_now != -1) {
      // speed_linear = c.substring(0, index_now).toFloat();
      // angular_speed = (c.substring(index_now + 1).toFloat());
      speed_desired[LEFT] = c.substring(0, index_now).toInt();
      speed_desired[RIGHT] = c.substring(index_now + 1).toInt();
      // int left_sp = c.substring(0, index_now).toInt();
      // int right_sp = c.substring(index_now + 1).toInt();
      // control_motor(0, left_sp);
      // control_motor(1, right_sp);
      // Serial.print("speed:");

      // Serial.print(left_sp);
      // erial.print(",");
      // Serial.println(right_sp);
      return 1;
    } else if (index_kp_desired != -1) {
      int index_cal = c.indexOf("#");
      if (index_cal != -1) {
        float new_kp = c.substring(0, index_kp_desired).toFloat();
        float new_ki = c.substring(index_kp_desired + 1, index_cal).toFloat();
        float new_kd = c.substring(index_cal + 1).toFloat();

        pid[LEFT].setParams(new_kp, new_ki, new_kd, 255);
        pid[RIGHT].setParams(new_kp, new_ki, new_kd, 255);

        Serial.print(pid[RIGHT].GetKp());
        Serial.print(" ");
        Serial.print(pid[RIGHT].GetKi());
        Serial.print(" ");
        Serial.println(pid[RIGHT].GetKd());
      }
    }
    else if(config_param!=-1) 
    // 0.2222L0.333R0.555W;
    {
      int index_R=c.indexOf("R");
      m_per_count_l=c.substring(0,config_param).toDouble();
      m_per_count_r=c.substring(config_param+1,index_R).toDouble();
      robot_width=c.substring(index_R+1).toDouble();
      Serial.print("CONFIG:");
      Serial.print(m_per_count_l,3);
      Serial.print(" ");
      Serial.print(m_per_count_r,3);
      Serial.print(" ");
      Serial.println(robot_width);
    }
    if(reset_index!=-1)
    {
      Serial.println("RESET");
      resetOdom();
    }
    if(IO_index!=-1)
    {
      // 1I0 -> broom -> 2I0 BLOOM --> 3I esc
      int IO_Function=c.substring(0,IO_index).toInt();
      int IO_state=c.substring(IO_index+1).toInt();
      switch(IO_Function)
      {
        case 0:
          BROOM(IO_state);
          break;
        case 1:
          BLOOM(IO_state);
          break;
        case 2:
          ServoControl(IO_state);
          break;
        case 3:
          ESControl(IO_state);
          break;
        case 4:
          BROOM(0);
          BLOOM(0);
          ServoControl(0);
          ESControl(0);
          break;
      }
    }
  }
  return 0;
}
void BROOM(int t) {
  if (1 == t) {
    digitalWrite(BRU_1, 0);
    digitalWrite(BRU_2, 1);
  } else if (0 == t) {
    digitalWrite(BRU_1, 0);
    digitalWrite(BRU_2, 0);
  }
}
void BLOOM(int t) {
  if (1 == t) {
    digitalWrite(BRD_1, 1);
    digitalWrite(BRD_2, 0);
  } else if (0 == t) {
    digitalWrite(BRD_1, 0);
    digitalWrite(BRD_2, 0);
  }
}
void startESCandServo() {
  SV1.attach(SV);  // 180 lau 140 dong
  delay(100);
  ESC1.attach(ESC);
  delay(100);

  ESC1.write(90);
  delay(200);
  ESC1.write(120);
  delay(200);
  ESC1.write(0);
  delay(2000);
}
void control_speed() {
  int delta_encoder[2] = { 0, 0 };
  /// lay so xung encoder

  // cli();
  for (int i = 0; i < 2; i++) {
    delta_encoder[i] = encoder_count[i];
    encoder_count[i] = 0;
  }
  // sei();
  ////tinh van toc
  int m_pwm[2] = { 0, 0 };
  float speed_filter[2];
 
  for (int i = 0; i < 2; i++) {

    publish_encoder[i] += delta_encoder[i];  // cong don encoder de tinh vi tri
    speed_filter[i] = delta_encoder[i] * 0.23905722 + last_encoder[i] * 0.23905722 + speed_filter[i] * 0.52188555;
    last_encoder[i] = delta_encoder[i];
    m_pwm[i] += pid[i].compute(delta_encoder[i], speed_desired[i], delta_time);  // speed >0 ->  delta must >0 pid
  }
  m_pwm[LEFT]=-m_pwm[LEFT];
  // Serial.print(publish_encoder[0]);
  // Serial.print(",");
  //   Serial.println(publish_encoder[1]);
  if (ros_serial) {
    for (int i = 0; i < 2; i++) {
      control_motor(i, m_pwm[i]);
    }
  } else {
    // delay(200);
    static bool start_run_motor = 0;
    if (receive_uart() && start_run_motor == 0) {
      start_run_motor = 1;
      for (int i = 0; i < 2; i++) pid[i].reset_all();
    }
    if (start_run_motor == 1) {
      static uint16_t count_print = 0;
      count_print += 1;
      Serial.print(delta_encoder[LEFT]);
      Serial.print(",");
      Serial.println(delta_encoder[RIGHT]);
      control_motor(LEFT, m_pwm[LEFT]);
      control_motor(RIGHT, m_pwm[RIGHT]);
      //  control_motor(LEFT, speed_desired[LEFT]);
      // control_motor(RIGHT, speed_desired[RIGHT]);
      if (count_print >= 100) {
        count_print = 0;
        speed_desired[0] = 0;
        speed_desired[1] = 0;
        start_run_motor = 0;
        control_motor(0, 0);
        control_motor(1, 0);
        Serial.println("done");
      }
    }
  }
}
void ServoControl(int state)
{
  if(state)SV1.write(180);
  else SV1.write(160);
}
void ESControl(int state)
{
  if(state)ESC1.write(120);
  else ESC1.write(0);
}
void resetOdom()
{
  robotGlobalPos.x=0;
  robotGlobalPos.y=0;
  robotGlobalPos.theta=0;
}
void send_odom() {
  // static unsigned long time_last=micros();
  // publish_time=(micros()-time_last)/1000;
  // time_last=micros();
  double delta_l=publish_encoder[LEFT]*m_per_count_l;
  double delta_r=publish_encoder[RIGHT]*m_per_count_r;
  double dxy=(delta_l+delta_r)/2.0;
  double dtheta=((delta_r-delta_l))/robot_width;
  double dx=dxy*cos(robotGlobalPos.theta+(dtheta/2.0));
  double dy=dxy*sin(robotGlobalPos.theta+(dtheta/2.0));
  robotGlobalPos.x+=dx;
  robotGlobalPos.y+=dy;
  robotGlobalPos.theta+=dtheta;
  // 5x10y0.9;
  Serial.print(robotGlobalPos.x*100,3);
  Serial.print("x");
  Serial.print(robotGlobalPos.y*100,3);
  Serial.print("y");
  Serial.print(robotGlobalPos.theta*180.0/PI);
  Serial.print("v");
  Serial.println(dxy);

  // Serial.print("*");
  // Serial.println(publish_time);
  publish_encoder[0] = 0;
  publish_encoder[1] = 0;
}
void setup() {
  Serial.begin(57600);
  pinMode(SV, OUTPUT);
  pinMode(ESC, OUTPUT);
  pinMode(BRU_1, OUTPUT);
  pinMode(BRU_2, OUTPUT);

  pinMode(BRD_1, OUTPUT);
  pinMode(BRD_2, OUTPUT);
  startESCandServo();
  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT_PULLUP);
    pinMode(encb[k], INPUT_PULLUP);
    pinMode(pwm[k], OUTPUT);
    pinMode(dir[k], OUTPUT);
    pinMode(dir2[k], OUTPUT);
  }
  for (int i = 0; i < 2; i++) {
    control_motor(i, 0);
  }
  pid[LEFT].setParams(7.8, 22.8, 0, 255);   //39.2 34.6
  pid[RIGHT].setParams(7.8, 22.8, 0, 255);  //39.2 34.6
  attachInterrupt(digitalPinToInterrupt(enca[LEFT]), encoderLeftMotor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[RIGHT]), encoderRightMotor, CHANGE);
  publish_time = micros();
}
void loop() {
  static unsigned long time_loop_pid = millis();
  static unsigned long time_publish = millis();
  static int last_a_left = 0;
  static int last_a_right = 0;
  static int last_b_left = 0;
  static int last_b_right = 0;
  int delta_now = 0;
  if (ros_serial) receive_uart();
  // Serial.println("123");
  // delay(500);
  // bool encb_l_now = digitalRead(encb[LEFT]);
  // bool encb_r_now = digitalRead(encb[RIGHT]);
  // if (PinStateChanged(encb_l_now, &last_a_left, &last_b_left)) {
  //   bool enca_now=digitalRead(enca[LEFT]);
  //   if (last_b_left == 0) { // rising
  //     delta_now = enca_now > 0 ? dir_encod_M0 : -dir_encod_M0;
  //   } else { // faling
  //     delta_now = enca_now > 0 ? -dir_encod_M0 : dir_encod_M0;
  //   }
  //   encoder_count[LEFT] += delta_now;
  // }
  // if (PinStateChanged(encb_r_now, &last_a_right, &last_b_right)) {
  //    bool enca_now=digitalRead(enca[RIGHT]);
  //   if (last_b_right == 0) {
  //     delta_now = enca_now > 0 ? dir_encod_M1 : -dir_encod_M1;
  //   } else {
  //     delta_now = enca_now > 0 ? -dir_encod_M1 : dir_encod_M1;
  //   }
  //   encoder_count[RIGHT] += delta_now;
  // }
  // if (millis() - time_loop_pid >= LOOP_MS) {
  //   control_speed();
  //   // Serial.println("123");
 
  //   time_loop_pid = millis();
  // }
  callFunctionPeriodically(control_speed, LOOP_MS, time_loop_pid);  // tinh pid van toc va encoder
  callFunctionPeriodically(send_odom,LOOP_PUB,time_publish); // gui len pi neu dung ros serial
}