
//mode: 
//0 = hbridge (PWMA and PWMB for different directions)
//1 = motor driver (PWMA for speed, IN1 and IN2for different directions)
int mode = 1;

//pins for moving the motor
#define PWM_BOT_A 11  
#define PWM_BOT_B 10  
#define IN_BOT_1 12   
#define IN_BOT_2 13   

//pins for moving the motor
#define PWM_TOP_A 6  
#define PWM_TOP_B 5  
#define IN_TOP_1 7   
#define IN_TOP_2 8   
/********************************************************************************/
void setup()
{

  //set pwm pins as outputs
  pinMode(PWM_BOT_A, OUTPUT);
  if (mode == 0) {
    pinMode(PWM_BOT_B, OUTPUT);
  } else if (mode == 1) {
    pinMode(IN_BOT_1, OUTPUT);
    pinMode(IN_BOT_2, OUTPUT);
    
  pinMode(PWM_TOP_A, OUTPUT);
  if (mode == 0) {
    pinMode(PWM_TOP_B, OUTPUT);
  } else if (mode == 1) {
    pinMode(IN_TOP_1, OUTPUT);
    pinMode(IN_TOP_2, OUTPUT);
  };
  }

  Serial.begin(9600);           //begin serial communication with the computer

 }

/********************************************************************************/
void loop()
{
  moveMotor_BOT(75); //change value between 0-255 for different speed
  moveMotor_TOP(115);
  delay(500); //delay for 500ms
  moveMotor_BOT(-75);
  moveMotor_TOP(-115);
  delay(500);
}

void moveMotor_BOT(float pwmVal) {
  int motorSpeed = (int)fabs(pwmVal);
  if (motorSpeed > 255) {  //constrain motorspeed value to max
    motorSpeed = 255;
  }

  if (pwmVal > 0)  
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

void moveMotor_TOP(float pwmVal) {
  int motorSpeed = (int)fabs(pwmVal);
  if (motorSpeed > 255) {  //constrain motorspeed value to max
    motorSpeed = 255;
  }

  if (pwmVal > 0)  
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

