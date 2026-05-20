#include "config.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// #include <util/atomic.h>
// #include "digitalWriteFast.h"
double m_per_count_l = 0;
double m_per_count_r = 0;
double robot_width = 1;

// right ~ 85 count/ vong
// left ~ 720
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
volatile int speed_desired[] = { 0, 0 };
volatile int encoder_count[NMOTORS];
long publish_encoder[NMOTORS];
float last_encoder[NMOTORS];
unsigned long publish_time = 0;
volatile int pos[NMOTORS] = { 0, 0 };
SimplePID pid[NMOTORS];
int encod_state[NMOTORS];
robot_pos robotGlobalPos;
/// @brief
/// @param motor
/// @param speed
void control_motor(int motor, int speed) {
  if (motor == RIGHT)
    speed = -speed;
  bool direct = speed > 0 ? 1 : 0;
  speed = constrain(speed, -255, 255);

  if (speed == 0) {
    analogWrite(pwm[motor], 0);
    digitalWrite(dir[motor], 0);
    return;
  }
  // digitalWrite(dir[motor],direct);
  if (direct) {
    analogWrite(pwm[motor], 255 - abs(speed));
    digitalWrite(dir[motor], 1);
  } else {
    analogWrite(pwm[motor], abs(speed));
    digitalWrite(dir[motor], 0);
  }
}
/// @brief  Display:
void lcd_print(String line1, String line2) {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(line1);

  lcd.setCursor(0, 1);
  lcd.print(line2);
}
/// @return
/// @brief Sensor read
void calibIMU() {
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  lcd_print("Calib IMU", "Do not move");

  Serial.println("Calib gyro...");

  const int num_samples = 2000;

  float gx = 0;
  float gy = 0;
  float gz = 0;

  for (int i = 0; i < num_samples; i++) {
    mpu.getEvent(&a, &g, &temp);

    gx += g.gyro.x;
    gy += g.gyro.y;
    gz += g.gyro.z;

    delay(2);
  }

  gyro_offset_x = gx / num_samples;
  gyro_offset_y = gy / num_samples;
  gyro_offset_z = gz / num_samples;

  Serial.println("Done calib");

  lcd_print("IMU Ready", "");
}
void readIMU() {
  mpu.getEvent(&a, &g, &temp);

  gyro_x = g.gyro.x - gyro_offset_x;
  gyro_y = g.gyro.y - gyro_offset_y;  // rad/s
  gyro_z = g.gyro.z - gyro_offset_z;
}
float readUltrasonic() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 30000);

  float distance = duration * 0.034 / 2.0;

  return distance;
}
bool readIR() {
  return digitalRead(IR);
}
bool readButton1() {
  return digitalRead(BT1);
}

bool readButton2() {
  return digitalRead(BT2);
}
void setupSensor() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(IR, INPUT);

  pinMode(BT1, INPUT_PULLUP);
  pinMode(BT2, INPUT_PULLUP);

  pinMode(LED1, OUTPUT);
  // Wire.begin();
  // delay(100);
  // Wire.setClock(100000);

  // Serial.println("START");

  // while (1) {
  //   if (mpu.begin(0x68, &Wire)) {
  //     break;
  //   }
  //   Serial.println("MPU6050 fail");
  //   delay(800);
  // }
  lcd.init();
  lcd.backlight();
  lcd_print("Robot Init", "");
  // calibIMU();
}
/// @param



/// @brief  send serial
void send_sensor() {
  ///  encL/encR/gz/us/ir/b1/b2;
  unsigned long time_delay = millis();
  // readIMU();

  float distance = readUltrasonic();

  int ir_state = digitalRead(IR);

  int bt1 = digitalRead(BT1);
  int bt2 = digitalRead(BT2);

  String sensor_data =
    String(publish_encoder[LEFT]) + "/" + String(publish_encoder[RIGHT]) + "/" + String(gyro_z, 4) + "/" + String(distance, 1) + "/" + String(ir_state) + "/" + String(bt1) + "/" + String(bt2) + ";"; /// bt1 = green --> press = 0

  Serial.println(sensor_data);
  // Serial.println("delay: "+String(millis()-time_delay));
}
/// @return
void IRAM_ATTR encoderLeftMotor() {
  static bool old_a = false;
  bool newA = digitalRead(enca[LEFT]);
  bool newB = digitalRead(encb[LEFT]);
  int delta = 0;

  if (newA && !old_a) {
    delta = newB > 0 ? dir_encod[LEFT] : -dir_encod[LEFT];
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod[LEFT] : dir_encod[LEFT];
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
    delta = newB > 0 ? dir_encod[RIGHT] : -dir_encod[RIGHT];
  }  // rising
  else {
    delta = newB > 0 ? -dir_encod[RIGHT] : dir_encod[RIGHT];
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
    if (index_now != -1) {
      // int index_now_2 = c.indexOf("*");
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

        for (int i = 0; i < NMOTORS; i++) {
          pid[i].setParams(new_kp, new_ki, new_kd, 255);
        }

        Serial.print(pid[M_L_UP].GetKp());
        Serial.print(" ");
        Serial.print(pid[M_L_UP].GetKi());
        Serial.print(" ");
        Serial.println(pid[M_L_UP].GetKd());
      }
    }
  }
  return 0;
}
void control_speed() {
  int delta_encoder[NMOTORS] = { 0, 0 };
  /// lay so xung encoder

  // cli();
  for (int i = 0; i < NMOTORS; i++) {
    delta_encoder[i] = encoder_count[i];
    encoder_count[i] = 0;
  }
  // sei();
  ////tinh van toc
  int m_pwm[NMOTORS] = { 0, 0 };
  float speed_filter[NMOTORS];

  for (int i = 0; i < NMOTORS; i++) {

    publish_encoder[i] += delta_encoder[i];  // cong don encoder de tinh vi tri
    speed_filter[i] = delta_encoder[i] * 0.23905722 + last_encoder[i] * 0.23905722 + speed_filter[i] * 0.52188555;
    last_encoder[i] = delta_encoder[i];
    m_pwm[i] = pid[i].compute(delta_encoder[i], speed_desired[i], delta_time);  // speed >0 ->  delta must >0 pid
  }
  // m_pwm[LEFT] = -m_pwm[LEFT];
  // Serial.print(publish_encoder[0]);
  // Serial.print(",");
  //   Serial.println(publish_encoder[1]);
  if (ros_serial) {
    for (int i = 0; i < NMOTORS; i++) {
      control_motor(i, m_pwm[i]);
    }
  } else {
    // delay(200);
    static bool start_run_motor = 0;
    if (receive_uart() && start_run_motor == 0) {
      start_run_motor = 1;
      for (int i = 0; i < NMOTORS; i++)
        pid[i].reset_all();
    }
    if (start_run_motor == 1) {
      static uint16_t count_print = 0;
      count_print += 1;
      Serial.print(delta_encoder[LEFT]);
      Serial.print(",");
      Serial.println(delta_encoder[RIGHT]);
      for (int i = 0; i < NMOTORS; i++) {
        control_motor(i, m_pwm[i]);
      }
      //  control_motor(LEFT, speed_desired[LEFT]);
      // control_motor(RIGHT, speed_desired[RIGHT]);
      if (count_print >= 200) {
        count_print = 0;
        speed_desired[0] = 0;
        speed_desired[1] = 0;
        start_run_motor = 0;
        for (int i = 0; i < NMOTORS; i++)
          control_motor(i, 0);
        Serial.println("done");
      }
    }
  }
}
void send_odom() {

  String odom_data = String(publish_encoder[LEFT]) + "/" + String(publish_encoder[RIGHT]) + ";";
  Serial.println(odom_data);
  // for(int i=0;i<NMOTORS;i++){
  //   publish_encoder[i]=0;
  // }
}
void setup() {
  Serial.begin(57600);
  for (int k = 0; k < NMOTORS; k++) {
    pinMode(enca[k], INPUT_PULLUP);
    pinMode(encb[k], INPUT_PULLUP);
    pinMode(pwm[k], OUTPUT);
    pinMode(dir[k], OUTPUT);
    pid[k].setParams(p_gain_default, i_gain_default, d_gain_default, 255);
    // pinMode(dir2[k], OUTPUT);
  }
  pinMode(13,OUTPUT);
  pinMode(25,OUTPUT);
  digitalWrite(13,HIGH);
  digitalWrite(25,HIGH);
  for (int i = 0; i < 2; i++) {
    control_motor(i, 0);
  }
  setupSensor();
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
  // control_motor(0,-5u0);
  if (ros_serial)
    receive_uart();
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
  callFunctionPeriodically(send_sensor, LOOP_PUB, time_publish);    // gui len pi neu dung ros serial
}