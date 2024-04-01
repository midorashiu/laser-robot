
//mode: 
//0 = hbridge (PWMA and PWMB for different directions)
//1 = motor driver (PWMA for speed, IN1 and IN2for different directions)
int mode = 0;

//pins for moving the motor
#define PWMA 11  
#define PWMB 10  
#define IN1 12   
#define IN2 13   

/********************************************************************************/
void setup()
{

  //set pwm pins as outputs
  pinMode(PWMA, OUTPUT);
  if (mode == 0) {
    pinMode(PWMB, OUTPUT);
  } else if (mode == 1) {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
  }

  Serial.begin(9600);           //begin serial communication with the computer

 }

/********************************************************************************/
void loop()
{
  moveMotor(100); //change value between 0-255 for different speed
  delay(500); //delay for 500ms
  moveMotor(-100);
  delay(500);
}

void moveMotor(float pwmVal) {
  int motorSpeed = (int)fabs(pwmVal);
  if (motorSpeed > 255) {  //constrain motorspeed value to max
    motorSpeed = 255;
  }

  if (pwmVal > 0)  
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
