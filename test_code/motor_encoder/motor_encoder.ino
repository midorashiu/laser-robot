// Define rotary encoder pins
#define ENC_A 2
#define ENC_B 3

#define PWMA 11
#define PWMB 10

static int lastCounter = 0;
unsigned long _lastIncReadTime = micros(); 
unsigned long _lastDecReadTime = micros(); 
int _pauseLength = 25000;
int _fastIncrement = 1;

volatile int counter = 0;

// PID constants
float kp = 1;
float kd = 0.025;
float ki = 0.0;

//vars for PID
int pos = 0; 
long prevT = 0;
float eprev = 0;
float eintegral = 0;

int target_positions = [8, 16, 8, 0];
int target;
int n = 0;

void setup() {
  // Start the serial monitor to show output
  Serial.begin(115200);
  // Set encoder pins and attach interrupts
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(PWMA,OUTPUT);
  pinMode(PWMB,OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_A), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), read_encoder, CHANGE); 
  
  Serial.println("target pos");
}

void loop() {

  static int lastCounter = 0;

  // If count has changed print the new value to serial
  if(counter != lastCounter){
    Serial.println(counter);
    lastCounter = counter;
  }

  //set target position
  target = target_positions[n];
  if (pos==target){ //once we reach a position, go to next position
    if (n<4) n++;
    else n=0;
  }

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  
  //read position
  pos = counter;

  //PID
  int e = pos - target; // error
  float dedt = (e-eprev)/(deltaT); // derivative
  eintegral = eintegral + e*deltaT; // integral

  // control signal
  float pwmVal = kp*e + kd*dedt + ki*eintegral;
  moveMotor(pwmVal);
  eprev = e;
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void moveMotor(float pwmVal){
  float motorSpeed = constrain(abs(pwmVal), 0, 255)
  if (pwmVal>0)                    
  {
    analogWrite(PWMA, motorSpeed); 
    analogWrite(PWMB, 0); 
  }
  else if(pwmVal<0)
  {
    analogWrite(PWMA, 0); 
    analogWrite(PWMB, motorSpeed); 
  }
  else
  {
    analogWrite(PWMA, 0); 
    analogWrite(PWMB, 0); 
  }
}
void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent
 
  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(ENC_A)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(ENC_B)) old_AB |= 0x01; // Add current state of pin B
  
  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if((micros() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((micros() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = micros();
    counter = counter + changevalue;              // Update counter
    encval = 0;
  }
}