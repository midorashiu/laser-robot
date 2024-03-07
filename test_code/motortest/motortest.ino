
//the bot motor will be controlled by the motor A pins on the motor driver
const int ENA = 13;           
const int ENB = 12;            //control pin 2 on the motor driver for the bot motor

const int PWMA1 = 11;            //speed control pin on the motor driver for the bot motor forward
const int PWMA2 = 10;            //speed control pin on the motor driver for the bot motor backwards

const int PWMB1 = 9;            //speed control pin on the motor driver for the bot motor forward
const int PWMB2 = 6;            //speed control pin on the motor driver for the bot motor backwards


const int driveTime = 20;      //this is the number of milliseconds that it takes the robot to drive 1 inch
                               //it is set so that if you tell the robot to drive forward 25 units, the robot drives about 25 inches

const int turnTime = 8;        //this is the number of milliseconds that it takes to turn the robot 1 degree
                               //it is set so that if you tell the robot to turn right 180 units, the robot turns about 180 degrees

                               //Note: these numbers will vary a little bit based on how you mount your motors, the friction of the
                               //surface that your driving on, and fluctuations in the power to the motors.
                               //You can change the driveTime and turnTime to make them more accurate

                               

String motor;           //the motor
String direction;       //the direction to travel

/********************************************************************************/
void setup()
{

  //set the motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(PWMA1, OUTPUT);
  pinMode(PWMA2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(PWMB1, OUTPUT);
  pinMode(PWMB2, OUTPUT);

  Serial.begin(9600);           //begin serial communication with the computer
  
  //enable h-bridge
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}

/********************************************************************************/
void loop()
{
  //test bot motor
  botMotor_forward(100);                        //bot motor clockwise
  delay(1000);                   //turn the bot motor 180 degrees
  botMotor_backwards(100);                       //drive the bot motor counter clockwise
  delay(1000);                 //turn the bot motor 180 degrees
  botMotor_stop();                       
  delay(1000);

  topMotor_forward(100);                        //drive the bot motor clockwise
  delay(1000);                   //turn the bot motor 180 degrees
  topMotor_backwards(100);                       //drive the bot motor counter clockwise
  delay(1000);                   //turn the bot motor 180 degrees
  topMotor_stop();                            //turn the bot motor off
  delay(1000);

}
/********************************************************************************/
void botMotor_forward(int motorSpeed)                       //function for driving the bot motor
{
    analogWrite(PWMA1, abs(motorSpeed)); 
    analogWrite(PWMA2, 0); 
}
void botMotor_backwards(int motorSpeed){
    analogWrite(PWMA1, 0); 
    analogWrite(PWMA2, abs(motorSpeed)); 
}
void botMotor_stop()                                             //if the motor should stop
{
    analogWrite(PWMA1, 0); 
    analogWrite(PWMA2, 0); 
}

/********************************************************************************/
void topMotor_forward(int motorSpeed)                       //function for driving the bot motor
{
    analogWrite(PWMB1, abs(motorSpeed)); 
    analogWrite(PWMB2, 0); 
}
void topMotor_backwards(int motorSpeed)                             //if the motor should drive backward (negative speed)
{
    analogWrite(PWMB1, 0); 
    analogWrite(PWMB2, abs(motorSpeed)); 
}
void topMotor_stop()                                              //if the motor should stop
{
    analogWrite(PWMB1, 0); 
    analogWrite(PWMB2, 0); 
}
