class SimplePID{
  private:
    float kp, kd, ki, umax;
    float eprev, eintegral,last_u;
  public:
    SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0) {}
     void reset_all()
    {
        eintegral=0;
        eprev=0;
    } 
    void setParams(float kpIn, float kiIn, float kdIn, float umaxIn){
      kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
      reset_all();
    }
    float compute(int value, int target, float deltaT){
      if(target==0){
        reset_all();
        return 0;
      }
      int e = target - value;
      float dedt = (e-eprev)/(deltaT);
      if(abs((int)last_u) >= umax && (((e >= 0) && (eintegral >= 0)) || ((e < 0) && (eintegral < 0))))
      {
        eintegral = eintegral;
      }
      else{
        eintegral+=e*deltaT;
      }
      eintegral=constrain(eintegral,-120,120);
      float u = kp*e + kd*dedt + ki*eintegral;
      if( u > umax)u = umax;
      else if(u<-umax)u=-umax;
      last_u=u;
      eprev = e;
      return u;
    }
    float GetKp(){return kp;}
    float GetKi(){return ki;}
    float GetKd(){return kd;}
   
};
typedef void (*CallbackFunction)();
void callFunctionPeriodically(CallbackFunction functionToCall, unsigned long intervalTime, unsigned long &previousMillis) {
  unsigned long currentMillis = millis();  //
  if (currentMillis - previousMillis >= intervalTime) {
    functionToCall();
    previousMillis = currentMillis;
  }
}
#define NMOTORS 2
#define LEFT 0
#define RIGHT 1
#define LOOP_PUB 100 //ms
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
#define ros_serial 0
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
const int enca[] = {25,32};
const int encb[]= {33,35}; 
const int pwm[] = { 26, 2 };  //{10,11}
const int dir[] = { 14, 16 };
const int dir2[] = { 27, 4 };
const int dir_encod_M1=1;
const int dir_encod_M0=-1; 
const float LOOP_FREQUENCY=50.0; // hz
const float delta_time=1.0/LOOP_FREQUENCY;
const float LOOP_MS=1000.0*delta_time;
const float speed_ff=10.0; // 100 Hz
const bool serial_tune = 0;
const int time_run_test=2000;
// const float MM_PER_COUNT_LEFT = (1 - ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
// const float MM_PER_COUNT_RIGHT = (1 + ROTATION_BIAS) * PI * WHEEL_DIAMETER / (ENCODER_PULSES * GEAR_RATIO);
const unsigned long micros_interval=(1.0/LOOP_FREQUENCY)*1e6;
const double PI=3.141592653589793;
