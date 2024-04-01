//mode: 0 = hbridge, 1 = motor driver
int mode = 1;
// Define rotary encoder pins
#define ENC_A 2
#define ENC_B 3
//pins for moving the motor
#define PWMA 11  //pwm for forward
#define PWMB 10  //pwm for backwards
#define IN1 12   //pwm for backwards
#define IN2 13   //pwm for backwards
//pin for ISR timing test
#define DUTY 7

//decoder variables
static int lastCounter = 0;
unsigned long _lastIncReadTime = micros();
unsigned long _lastDecReadTime = micros();
int _pauseLength = 25000;
int _fastIncrement = 1;
volatile int counter = 0;

// PID constants
float kp = 1.2;//5.201873;
float kd = 0;//0.017286;
float ki = 0;//1.772212;

//vars for PID
int pos = 0;
long prevT = 0;
float errprev = 0;
int err;
float deriv;
float derivFilt;
float integ = 0;
float pwmVal;

int target;

//filter variables
const int numSamples = 5;  //number of samples for filtering
float weightRaw[numSamples];
float weightNorm[numSamples];
float weightSum = 0;
float dataFilt[numSamples];
float filtSum = 0;
int sampleCount = 0;

//for testing
volatile unsigned targetTime = 0;
bool pulse = true;

// Timer 1 compare interrupt service routine
ISR(TIMER1_COMPA_vect){
  //turn on GPIO pin for ISR timing test
  digitalWrite(DUTY, HIGH);
  //advance the COMPA Register
  OCR1A += 1707;
  //calculate vout from current values
  pwmVal = kp * err + kd * derivFilt + ki * integ;
  //move the motor
  moveMotor(pwmVal);
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
  memset(dataFilt, 0, sizeof(dataFilt)); //reset filtered data
}

void loop() {

  //set 
  static int lastCounter = 0;
  // If count has changed print the new value to serial
  if (counter != lastCounter) {
    lastCounter = counter;
  }

  setTarget();

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  //continuously get values
  pos = counter;
  err = pos - target;                  // error
  deriv = (err - errprev) / (deltaT);  // derivative
  weightedFilter(deriv);
  integ = integ + err * deltaT;  // integral
  errprev = err;

  debugStatements();
}
void debugStatements(){
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(pwmVal);
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
  // Set encoder pins and attach interrupts
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE);
  //set pwm pins as outputs
  pinMode(PWMA, OUTPUT);
  if (mode == 0) {
    pinMode(PWMB, OUTPUT);
  } else if (mode == 1) {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
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

  //target = 100;
  if (millis() - targetTime >= 1000) {  // Change target every 5 second
    targetTime = millis();
    if (pulse) target += 100;
    else target -= 100;
    pulse = !pulse;
  }
}

void weightedFilter(float data) {
  filtSum -= dataFilt[sampleCount];                       //remove oldest entry from sum
  dataFilt[sampleCount] = data * weightNorm[sampleCount];  //update entry with new data
  filtSum += dataFilt[sampleCount];
  derivFilt = filtSum / (float)numSamples;
  sampleCount = (sampleCount + 1) % numSamples;
}

void moveMotor(float pwmVal) {
  int motorSpeed = (int)fabs(pwmVal);
  if (motorSpeed > 255) {  //constrain motorspeed value to max
    motorSpeed = 255;
  }

  if (pwmVal > 0)  //going forward
  {
    if (mode == 0) {
      analogWrite(PWMA, motorSpeed);
      analogWrite(PWMB, 0);
    } else if (mode == 1) {
      analogWrite(PWMA, motorSpeed);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    }

  } else if (pwmVal < 0)  //going backwards
  {
    if (mode == 0) {
      analogWrite(PWMA, 0);
      analogWrite(PWMB, motorSpeed);
    } else if (mode == 1) {
      analogWrite(PWMA, motorSpeed);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }
  } else  //stop motor when pwmVal is 0
  {
    analogWrite(PWMA, 0);
    if (mode == 0) {
      analogWrite(PWMB, 0);
    }
  }
}
void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent

  static uint8_t old_AB = 3;                                                                  // Lookup table index
  static int8_t encval = 0;                                                                   // Encoder value
  static const int8_t enc_states[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };  // Lookup table

  old_AB <<= 2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02;  // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01;  // Add current state of pin B

  encval += enc_states[(old_AB & 0x0f)];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if (encval > 3) {  // Four steps forward
    int changevalue = 1;
    if ((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue;
    }
    _lastIncReadTime = micros();
    counter = counter + changevalue;  // Update counter
    encval = 0;
  } else if (encval < -3) {  // Four steps backward
    int changevalue = -1;
    if ((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue;
    }
    _lastDecReadTime = micros();
    counter = counter + changevalue;  // Update counter
    encval = 0;
  }
}