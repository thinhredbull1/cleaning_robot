class SimplePID {
private:
  float kp, kd, ki, umax;
  float eintegral, u, last_e_2,last_e;
public:
  SimplePID()
    : kp(1), kd(0), ki(0), umax(255), last_e(0.0), last_e_2(0.0) {}
  void reset_all() {
    last_e_2 = 0;
    last_e = 0;
    u=0;
  }
  void setParams(float kpIn, float kiIn, float kdIn, float umaxIn) {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
    reset_all();
  }
  float compute(int value, int target, float deltaT) {
    if (target == 0) {
      reset_all();
      return 0;
    }
    int e = target - value;
    float delta_u=kp*(e-last_e)+ki*e+kd*(e-2*last_e+last_e_2);
    
    u +=delta_u;
    if (u > umax) u = umax;
    else if (u < -umax) u = -umax;
    last_e_2=last_e;
    last_e=e;
    return u;
  }
  float GetKp() {
    return kp;
  }
  float GetKi() {
    return ki;
  }
  float GetKd() {
    return kd;
  }
};
typedef void (*CallbackFunction)();
void callFunctionPeriodically(CallbackFunction functionToCall, unsigned long intervalTime, unsigned long &previousMillis) {
  unsigned long currentMillis = millis();  //
  if (currentMillis - previousMillis >= intervalTime) {
    functionToCall();
    previousMillis = currentMillis;
  }
}
#define NMOTORS 4
#define LEFT 0
#define RIGHT 1
#define M_L_UP 0
#define M_L_DOWN 1 
#define M_R_UP 2
#define M_R_DOWN 3
#define LOOP_PUB 50 // 50ms
//macro for detection af rasing edge
#define RE(signal, state) (state=(state<<1)|(signal&1)&3)==1
//macro for detection af falling edge
#define FE(signal, state) (state=(state<<1)|(signal&1)&3)==2
bool PinStateChanged(int pin, int *lastButtonState, int *buttonRisingEdge) {
  //Get pin state
  int buttonState =pin;

  //Here starts the code for detecting an edge
  if (buttonState != *lastButtonState) {
    if (buttonState == LOW) {
      *buttonRisingEdge = 0;
    } else {
      *buttonRisingEdge = 1;
    }
    *lastButtonState = buttonState;
    return true;
  }

  return false;
}
typedef struct robot_coord{
  double x;
  double y;
  double theta;
}robot_pos;
#define ros_serial 1
// max speed 30 pulse / 100hz
//2030 pulse 1 step

#define SV 23
#define ESC 28
#define BRU_1 13
#define BRU_2 15

#define BRD_1 5
#define BRD_2 17

#define IR_R 34
#define IR_L 39
#define IR_U 36
const bool test_ff=0;
const int enca[] = {25,27,13,5};
const int encb[]= {33,26,14,15}; 
const int pwm[] = { 4,17,19,22 };  //{10,11}
const int dir[] = { 16,18 ,21,23};
const int dir_M_L_UP=1;
const int dir_M_L_DOWN=1;
const int dir_M_R_UP=-1;
const int dir_M_R_DOWN=-1;
int dir_encod[]={1,1,1,1};
float p_gain_default=15.15;
float i_gain_default=1.05;
float d_gain_default=1.0;
const int dir_encod_M1=1;
const int dir_encod_M0=-1; 
const float LOOP_FREQUENCY=100.0; // hz
const float delta_time=1.0/LOOP_FREQUENCY;
const float LOOP_MS=1000.0*delta_time; // 50 hz - 20ms
const float speed_ff=10.0; // 100 Hz
const bool serial_tune = 0;
const int time_run_test=2000;
// const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
// const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const unsigned long micros_interval=(1.0/LOOP_FREQUENCY)*1e6;
// const double PI=3.141592653589793;
