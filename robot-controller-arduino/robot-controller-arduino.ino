//mode: 0 = hbridge, 1 = motor driver
int mode = 1;
//------------------------------DEFINE PINS---------------------------------//
//Bottom motor rotary encoder pins
#define ENC_BOT_A 2 //Bottom motor encoder pin A
#define ENC_BOT_B 3 //Bottom motor encoder pin B
//Top motor rotary encoder pins
#define ENC_TOP_A 2 //Top motor encoder pin A
#define ENC_BTOP_B 3 //Top motor encoder pin B

//pins for moving the bottom motor
#define PWM_BOT_A 11  //bottom motor pwm for motor driver/hbridge (forward)
#define PWM_BOT_B 10  //bottom motor pwm for hbridge (backwards)
#define IN_BOT_1 12   //bottom motor direction for motor driver
#define IN_BOT_2 13   //bottom motor direction for motor driver
//pins for moving the top  motor
#define PWM_TOP_A 11  //top motor pwm for motor driver/hbridge (forward)
#define PWM_TOP_B 10  //top motor pwm for hbridge (backwards)
#define IN_TOP_1 12   //top motor direction for motor driver
#define IN_TOP_2 13   //top motor direction for motor driver

//pin for ISR timing test
#define DUTY 7
//------------------------------DECODER--------------------------------------//
 int encoder_BOT_pos;
 int encoder_TOP_pos;
//------------------------------PID------------------------------------------//
long prevT = 0;

// Bottom motor PID constants
float kp_BOT = 0.069207;//bottom motor proportional constant
float ki_BOT = 0.026327;//bottom motor integral constant
float kd_BOT = 0.039141;//bottom motor derivative constant
// Top motor PID constants
float kp_TOP = 0.069207;//top motor proportional constant
float ki_TOP = 0.026327;//top motor integral constant
float kd_TOP = 0.039141;//top motor derivative constant

//bottom motor PID variables
int pos_BOT = 0; //bottom motor position
int err_BOT; //bottom motor position error
float errprev_BOT = 0; //bottom motor previous position error
float deriv_BOT; //bottom motor derivative of position 
float derivFilt_BOT; //bottom motor filtered derivative of position 
float integ_BOT = 0; //bottom motor integral of position 
float pwmVal_BOT; //bottom motor pwm value 
//top motor PID variables
int pos_TOP = 0; //top motor position
int err_TOP; //top modataFilttor position error
float errprev_TOP = 0; //top motor previous position error
float deriv_TOP; //top motor derivative of position 
float derivFilt_TOP; //top motor filtered derivative of position 
float integ_TOP = 0; //top motor integral of position 
float pwmVal_TOP; //top motor pwm value 

//drawing target shape of square
int target;
int state = 0; //state of the shape 
int x_coords[4] = {0,0,10,10}; //x coordinates for bottom motor
int y_coords[4] = {0,10,10,0}; //y coordinates for top motor

//weighted sum filter variables
const int numSamples = 5;  //number of samples for filtering
float weightRaw[numSamples]; //raw weights
float weightNorm[numSamples]; //normalized weights
float weightSum = 0; //sum of the weights
int sampleCount = 0; //keeps track of the number of samples
//bottom motor filter
float dataFilt_BOT[numSamples]; //filtered data of the bottom motor
float filtSum_BOT = 0; //sum of filtered data bottom motor
//top motor filter
float dataFilt_TOP[numSamples];//filtered data of the top motor
float filtSum_TOP = 0; //sum of filtered data top motor

//for testing
volatile unsigned targetTime = 0;
bool pulse = true;

// Timer 1 compare interrupt service routine
ISR(TIMER1_COMPA_vect){
  //turn on GPIO pin for ISR timing test
  digitalWrite(DUTY, HIGH);
  //advance the COMPA Register
  OCR1A += 1707;
  //calculate vout for bottom motor from current values
  pwmVal_BOT = kp_BOT * err_BOT + kd_BOT * derivFilt_BOT + ki_BOT * integ_BOT;
   //calculate vout for top motor from current values
  pwmVal_TOP = kp_TOP * err_TOP + kd_TOP * derivFilt_TOP + ki_TOP * integ_TOP;
  //move the motors
  moveBottomMotor(pwmVal_BOT); //move the bottom motor
  moveTopMotor(pwmVal_TOP); //move the top motor
  //turn off GPIO pin for ISR timing test
  digitalWrite(DUTY, LOW);
}

void setup() {
  noInterrupts(); // Disable all interrupts
  init_timer1(); // Setup timer 1
  interrupts(); // Enable all interrupts
  init_pins();
  Serial.begin(9600);// Start the serial monitor to show output
  filter_coeffs(); //calculate the coefficients for the weighted sum filter
  memset(dataFilt_BOT, 0, sizeof(dataFilt_BOT)); //reset filtered data
  memset(dataFilt_TOP, 0, sizeof(dataFilt_TOP)); //reset filtered data
}

void loop() {

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  //move to the next vertex when there is a small position error
  if ((err_BOT<10) && (err_BOT<10)){
    if (state==3){ //restart the shape
      state = 0;
    }
    else{
      state++;
    }
  }

  //continuously get values for bottom motor
  pos_BOT = encoder_BOT_pos;
  pos_TOP = encoder_TOP_pos;
  err_BOT = pos_BOT - x_coords[state];                  // error
  err_TOP = pos_TOP - y_coords[state];  
  deriv_BOT = (err_BOT - errprev_BOT) / (deltaT);  // derivative
  deriv_TOP = (err_TOP - errprev_TOP) / (deltaT);
  weightedFilter(deriv_BOT, deriv_TOP);
  integ_BOT = integ_BOT + err_BOT * deltaT;  // integral
  integ_TOP = integ_TOP + err_TOP * deltaT;
  errprev_BOT = err_BOT;
  errprev_TOP = err_TOP;

  debugStatements();
}
void debugStatements(){
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(pwmVal);
  // //Serial.print(" ");
  // //Serial.print(err);
  // // Serial.print(" ");
  // Serial.print(deriv);
  // Serial.print(" ");
  // Serial.print(derivFilt);
  Serial.println();
  //}

}
//Initiates timer 1 and its interrupts
inline void init_timer1() {
  TCCR1A = 0; //Init Timer1A
  TCCR1B = 0; //Init TImer1B
  TCCR1B |= B00000001;  // Prescaler = 1
  OCR1A = 1707; //Set output compare value = 16MHz*107us
  TIMSK1 |= B00000010;  // Enable Timer Compare Interrupt
}
inline void init_pins(){
  // Set bottom motor encoder pins and attach interrupts
  pinMode(ENC_BOT_A, INPUT_PULLUP); 
  pinMode(ENC_BOT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_BOT_A), read_encoder_BOT, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_BOT_B), read_encoder_BOT, CHANGE);
  // Set top motor encoder pins and attach interrupts
  pinMode(ENC_TOP_A, INPUT_PULLUP);
  pinMode(ENC_TOP_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_TOP_A), read_encoder_TOP, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_TOP_B), read_encoder_TOP, CHANGE);
  //set pwm pins as outputs
  pinMode(PWM_BOT_A, OUTPUT);
  pinMode(PWM_TOP_A, OUTPUT);
  if (mode == 0) {
    pinMode(PWM_BOT_B, OUTPUT);
    pinMode(PWM_TOP_B, OUTPUT);
  } else if (mode == 1) {
    pinMode(IN_BOT_1, OUTPUT);
    pinMode(IN_BOT_2, OUTPUT);
    pinMode(IN_TOP_1, OUTPUT);
    pinMode(IN_TOP_2, OUTPUT);
  }
  //set pin for ISR timing test
  pinMode(DUTY, OUTPUT);
}
inline void filter_coeffs(){
  //compute filter coeff
  for (int n = 1; n <= numSamples; n++) {
    weightRaw[n - 1] = exp(-4.0 * (float)(n - 1) / (float)(numSamples - 1));
    weightSum += weightRaw[n - 1];
  }
  for (int n = 0; n < numSamples; n++) {
    weightNorm[n] = weightRaw[n] / weightSum;
  }
}
void setTarget(){

}

void weightedFilter(float data_BOT,float data_TOP) {
  filtSum_BOT -= dataFilt_BOT[sampleCount];  
  filtSum_TOP -= dataFilt_TOP[sampleCount];                     //remove oldest entry from sum
  dataFilt_BOT[sampleCount] = data_BOT * weightNorm[sampleCount];  //update entry with new data
  dataFilt_TOP[sampleCount] = data_TOP * weightNorm[sampleCount];
  filtSum_BOT += dataFilt_BOT[sampleCount];
  filtSum_TOP += dataFilt_TOP[sampleCount];
  derivFilt_BOT = filtSum_BOT / (float)numSamples;
  derivFilt_TOP = filtSum_TOP / (float)numSamples;
  sampleCount = (sampleCount + 1) % numSamples;
}

void moveBottomMotor(float pwmVal) {
  int motorSpeed = (int)fabs(pwmVal);
  if (motorSpeed > 255) {  //constrain motorspeed value to max
    motorSpeed = 255;
  }

  if (pwmVal > 0)  //going forward
  {
    if (mode == 0) {
      analogWrite(PWM_BOT_A, motorSpeed);
      analogWrite(PWM_BOT_B, 0);
    } else if (mode == 1) {
      analogWrite(PWM_BOT_A, motorSpeed);
      digitalWrite(IN_BOT_1, HIGH);
      digitalWrite(IN_BOT_2, LOW);
    }

  } else if (pwmVal < 0)  //going backwards
  {
    if (mode == 0) {
      analogWrite(PWM_BOT_A, 0);
      analogWrite(PWM_BOT_B, motorSpeed);
    } else if (mode == 1) {
      analogWrite(PWM_BOT_A, motorSpeed);
      digitalWrite(IN_BOT_1, LOW);
      digitalWrite(IN_BOT_2, HIGH);
    }
  } else  //stop motor when pwmVal is 0
  {
    analogWrite(PWM_BOT_A, 0);
    if (mode == 0) {
      analogWrite(PWM_BOT_B, 0);
    }
  }
}

void moveTopMotor(float pwmVal) {
  int motorSpeed = (int)fabs(pwmVal);
  if (motorSpeed > 255) {  //constrain motorspeed value to max
    motorSpeed = 255;
  }

  if (pwmVal > 0)  //going forward
  {
    if (mode == 0) {
      analogWrite(PWM_TOP_A, motorSpeed);
      analogWrite(PWM_TOP_B, 0);
    } else if (mode == 1) {
      analogWrite(PWM_TOP_A, motorSpeed);
      digitalWrite(IN_TOP_1, HIGH);
      digitalWrite(IN_TOP_2, LOW);
    }

  } else if (pwmVal < 0)  //going backwards
  {
    if (mode == 0) {
      analogWrite(PWM_TOP_A, 0);
      analogWrite(PWM_TOP_B, motorSpeed);
    } else if (mode == 1) {
      analogWrite(PWM_TOP_A, motorSpeed);
      digitalWrite(IN_TOP_1, LOW);
      digitalWrite(IN_TOP_2, HIGH);
    }
  } else  //stop motor when pwmVal is 0
  {
    analogWrite(PWM_TOP_A, 0);
    if (mode == 0) {
      analogWrite(PWM_TOP_B, 0);
    }
  }
}

void read_encoder_BOT() {
  encoder_BOT_pos; 
}
void read_encoder_TOP() {
  encoder_TOP_pos;
}