#include <PS2X_lib.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

//Motors
#define MOTOR_R_PWM 6
#define MOTOR_R_DIRECT 7
#define MOTOR_L_PWM 9
#define MOTOR_L_DIRECT 8

#define SPEED_MIN 215
#define SPEED_MAX 255
#define SPEED_STEP 1

#define DIRECT_FORWARD 1
#define DIRECT_BACKWARD 2

//Servos
#define SERVO_MIN  200 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_MAX  1000 // this is the 'maximum' pulse length count (out of 4096)

#define SERVO_CLAW 1 //servoClaw = 35 - 75%
#define SERVO_HAND_R 3 //servoHandR = 0 - 100%
#define SERVO_HAND_M 5 //servoHandM 10 - 90%
#define SERVO_ELBOW 7 //servoElbM 0 - 100%
#define SERVO_SHLD_M 9 //servoShldM 30 - 70%
#define SERVO_SHLD_R 11 //servoShldR 0 - 100%

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//PS2 Joystick
PS2X ps2x;

int eeAddress = 0;
byte servoPosition[] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50};
byte servoActive = 0;

struct ServoSetting {
  byte claw;
  byte handr;
  byte handm;
  byte elbow;
  byte shldm;
  byte shldr;  
};

byte error = 0; 
byte vibrate = 0;

int motorLSpeed = 0;
int motorRSpeed = 0;
byte motorLActive = 0;
byte motorRActive = 0;

//byte motorLDirect = 1;
//byte motorRDirect = 1;
byte counterSS = 0;

void setup()
{
  Serial.begin(57600);
 
  error = ps2x.config_gamepad(13,4,10,5, true, true);
 
  if(error == 0)
    Serial.println("Found Controller, configured successful");
  else if(error == 1)
    Serial.println("No controller found");   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands.");
  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  if (ps2x.readType() == 1)
  {
    Serial.println("DualShock Controller Found");
  } else {
    Serial.println("Unknown Controller Found");
  }
  Serial.println("Step 1"); 
  //Servo
  pwm.begin(); 
  pwm.setPWMFreq(100);  

Serial.println("Step 2");
  
  readServoSettings();
    
  //Motors
  pinMode(MOTOR_R_DIRECT, OUTPUT);
  pinMode(MOTOR_L_DIRECT, OUTPUT);

Serial.println("Step 3");

  delay(100);
}

void loop() 
{
  if(error == 1) //skip loop if no controller found
  {
    Serial.println("Error Joystick");
    return; 
  }
//Serial.println(".");

    ps2x.read_gamepad(false, vibrate);          //read controller and set large motor to spin at 'vibrate' speed

    motorLActive = 0;
    motorRActive = 0;
    servoActive = 0;
    
    if(ps2x.Button(PSB_START))
    {
      // Serial.println("Start");
      // 
    }
    if(ps2x.Button(PSB_SELECT))
    {
       Serial.println("Select");
       initServoTop();
    }

    if(ps2x.Button(PSB_GREEN))
    {
       Serial.println("Green");
       //initServoSerpent();
        //Serial.print("Claw pwm: "); Serial.println(pwm.getPWM(SERVO_CLAW));
        //Serial.print("Elbow pwm: "); Serial.println(pwm.getPWM(SERVO_ELBOW));
        //Serial.print("Main pwm: "); Serial.println(pwm.getPWM(SERVO_SHLD_M));
    }

    if(ps2x.Button(PSB_RED))
    {
        Serial.println("Red");
        counterSS++;
        writeServoSettings();
        Serial.println("Servo setting written");       
    }
    if(ps2x.Button(PSB_BLUE))
    {
       Serial.println("Blue");
    }
    if(ps2x.Button(PSB_PINK))
    {
       Serial.println("Pink");
    }

    controlButtonMotors();

    if(ps2x.Button(PSB_R1))
    {
      controlStikMotors1();
      //stikersLog();
    } 

    if(ps2x.Button(PSB_L1))
    {
      controlStikMotors2();
      //stikersLog();
    } 
   
    if(ps2x.Button(PSB_R2))
    {
      controlStikServosBottom();
      //stikersLog();
    } 

    if(ps2x.Button(PSB_L2))
    {
      controlStikServosTop();
      //stikersLog();
    }

    //by default motors speed down
    if ( motorLActive == 0 )
    {
        motorLStop();
    }
    if ( motorRActive == 0 )
    {
        motorRStop();
    } 

    if (servoActive == 1)
    {
      counterSS++;
    }

    if (counterSS > 10)
    {
      counterSS = 0;
    }
    
    if (counterSS == 10)
    {
      counterSS++;
      writeServoSettings();
      Serial.println("Servo setting written");
    }
    delay(50);
}

//////////////
// CONTROL  //
//////////////

void controlButtonMotors()
{
  if(ps2x.Button(PSB_PAD_UP)) 
  { 
    motorsForward();
    Serial.println("Up");
  }
  
  if(ps2x.Button(PSB_PAD_RIGHT))
  {
    motorsRight();
    Serial.println("Right");
  }
  
  if(ps2x.Button(PSB_PAD_LEFT))
  {
    motorsLeft();
    Serial.println("Left");
  }
  
  if(ps2x.Button(PSB_PAD_DOWN))
  {
    motorsBack();
    Serial.println("Down");
  }   

  /*if ( motorsActive == 0 )
  {
    motorLSpeedDown();
    motorRSpeedDown();
  }*/
}

void controlStikMotors1()
{
  if (ps2x.Analog(PSS_LY) > 130)
  {
    Serial.println("Stick Back");
    motorsBack();
  }
  else if (ps2x.Analog(PSS_LY) < 126)
  {
    Serial.println("Stick Forward");   
    motorsForward();
  }

  if (ps2x.Analog(PSS_LX) > 130)
  {
    motorsRight();
  }
  else if (ps2x.Analog(PSS_LX) < 126)
  {
    motorsLeft();
  }
}

void controlStikMotors2()
{
  if (ps2x.Analog(PSS_LY) > 130)
  {   
    motorLActive = 1; 
    motorLDown();
    
  }
  else if (ps2x.Analog(PSS_LY) < 126)
  {
    motorLActive = 1; 
    motorLUp();
  }

  if (ps2x.Analog(PSS_RY) > 130)
  {
    motorRActive = 1; 
    motorRDown();
  }
  else if (ps2x.Analog(PSS_RY) < 126)
  {
    motorRActive = 1; 
    motorRUp();
  }
}

void controlStikServosBottom()
{
  //SERVO_SHLD_M
  if (ps2x.Analog(PSS_LY) > 130)
  {
    moveServoUp(SERVO_SHLD_M);
  }
  else if (ps2x.Analog(PSS_LY) < 126)
  {
    moveServoDown(SERVO_SHLD_M);
  }

  //SERVO_SHLD_R
  if (ps2x.Analog(PSS_LX) > 130)
  {
    moveServoUp(SERVO_SHLD_R);
  }
  else if (ps2x.Analog(PSS_LX) < 126)
  {
    moveServoDown(SERVO_SHLD_R);
  }

  //SERVO_ELBOW
  if (ps2x.Analog(PSS_RY) > 130)
  {
    moveServoUp(SERVO_ELBOW);
  }
  else if (ps2x.Analog(PSS_RY) < 126)
  {
    moveServoDown(SERVO_ELBOW);
  }

  //SERVO_HAND_R
  if (ps2x.Analog(PSS_RX) > 130)
  {
    moveServoUp(SERVO_HAND_R);
  }
  else if (ps2x.Analog(PSS_RX) < 126)
  {
    moveServoDown(SERVO_HAND_R);
  }  
}
void controlStikServosTop()
{
  //SERVO_HAND_M
  if (ps2x.Analog(PSS_LY) > 130)
  {
    moveServoUp(SERVO_HAND_M);
  }
  else if (ps2x.Analog(PSS_LY) < 126)
  {
    moveServoDown(SERVO_HAND_M);
  }

  //SERVO_SHLD_R
  if (ps2x.Analog(PSS_LX) > 130)
  {
    moveServoUp(SERVO_SHLD_R);
  }
  else if (ps2x.Analog(PSS_LX) < 126)
  {
    moveServoDown(SERVO_SHLD_R);
  }  

  //SERVO_ELBOW
  if (ps2x.Analog(PSS_RY) > 130)
  {
    moveServoUp(SERVO_CLAW);
  }
  else if (ps2x.Analog(PSS_RY) < 126)
  {
    moveServoDown(SERVO_CLAW);
  }

  //SERVO_HAND_R
  if (ps2x.Analog(PSS_RX) > 130)
  {
    moveServoUp(SERVO_HAND_R);
  }
  else if (ps2x.Analog(PSS_RX) < 126)
  {
    moveServoDown(SERVO_HAND_R);
  }
}

void stikersLog()
{
  Serial.print("Stick Values:");
  Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
  Serial.print(",");
  Serial.print(ps2x.Analog(PSS_LX), DEC); 
  Serial.print("; ");
  Serial.print(ps2x.Analog(PSS_RY), DEC); 
  Serial.print(",");
  Serial.println(ps2x.Analog(PSS_RX), DEC); 
}

////////////
// MOTOR  //
////////////

void motorGo(uint8_t pinDirect, uint8_t pinSpeed, int speed)
{
  if (speed < 0)
  {
    digitalWrite(pinDirect, HIGH);
  }
  else {
    digitalWrite(pinDirect, LOW);
  }
  
  Serial.print("Speed "); Serial.print(pinSpeed); Serial.print(":"); Serial.println(abs(speed) + SPEED_MIN);

  if (speed == 0)
  {
    analogWrite(pinSpeed, 0);
  }
  else {
    analogWrite(pinSpeed, abs(speed) + SPEED_MIN);  
  }
  
}

//RIGHT

void motorRUp()
{
  if ( abs(motorRSpeed + SPEED_STEP) + SPEED_MIN <= SPEED_MAX )
  {
    motorRSpeed = motorRSpeed + SPEED_STEP;
    
    if (motorRSpeed < 0)
    {
      motorRSpeed = motorRSpeed + SPEED_STEP;
    }
    
  }
  
  motorGo(MOTOR_R_DIRECT, MOTOR_R_PWM, motorRSpeed);
}

void motorRDown()
{
  if ( abs(motorRSpeed - SPEED_STEP) + SPEED_MIN <= SPEED_MAX )
  {
    motorRSpeed = motorRSpeed - SPEED_STEP;   

    if (motorRSpeed > 0)
    {
      motorRSpeed = motorRSpeed - SPEED_STEP;
    }
  }
  
  motorGo(MOTOR_R_DIRECT, MOTOR_R_PWM, motorRSpeed);
}

void motorRStop()
{
  if (motorRSpeed > 0)
  {
    motorRDown();
  }
  else if (motorRSpeed < 0)
  {
    motorRUp();
  }
}


//LEFT

void motorLUp()
{
  if ( abs(motorLSpeed + SPEED_STEP) + SPEED_MIN <= SPEED_MAX )
  {
    motorLSpeed = motorLSpeed + SPEED_STEP; 

    if (motorLSpeed < 0)
    {
      motorLSpeed = motorLSpeed + SPEED_STEP;
    }
  }
  
  motorGo(MOTOR_L_DIRECT, MOTOR_L_PWM, motorLSpeed);
}

void motorLDown()
{
  if ( abs(motorLSpeed - SPEED_STEP) + SPEED_MIN <= SPEED_MAX )
  {
    motorLSpeed = motorLSpeed - SPEED_STEP;   

    if (motorLSpeed > 0)
    {
      motorLSpeed = motorLSpeed - SPEED_STEP;
    }
  }
  
  motorGo(MOTOR_L_DIRECT, MOTOR_L_PWM, motorLSpeed);
}

void motorLStop()
{
  if (motorLSpeed > 0)
  {
    motorLDown();
  }
  else if (motorLSpeed < 0)
  {
    motorLUp();
  }
}


//ALL MOTORS

void motorsForward()
{
  motorLActive = 1;
  motorRActive = 1;
  motorLUp();
  motorRUp();
}

void motorsBack()
{
  motorLActive = 1;
  motorRActive = 1;
  
  motorLDown();
  motorRDown();  
}

void motorsLeft()
{
  motorRActive = 1;
  
  motorLSpeed = SPEED_STEP;
  motorLStop();
  motorRUp();
}

void motorsRight()
{
  motorLActive = 1;
    
  motorRSpeed = SPEED_STEP;
  motorRStop();
  motorLUp();  
}

////////////
// SERVO  //
////////////

void initServoTop()
{
  moveServo(SERVO_SHLD_M, 50);      
  delay(100);
  moveServo(SERVO_ELBOW, 50);
  delay(100);  
  moveServo(SERVO_HAND_M, 50);
  delay(100);
  moveServo(SERVO_SHLD_R, 50);
  delay(100);  
  moveServo(SERVO_HAND_R, 50);
  delay(100);
  moveServo(SERVO_CLAW, 50); 
}

void initServoSerpent()
{
  moveServo(SERVO_SHLD_M, 70);      
  delay(500);
  moveServo(SERVO_ELBOW, 90);
  delay(500);
  moveServo(SERVO_HAND_M, 40);
  delay(500);

  moveServo(SERVO_SHLD_R, 50);
  delay(500);
  
  moveServo(SERVO_HAND_R, 50);
  delay(500);
    
  moveServo(SERVO_CLAW, 30); 
}

void moveServoUp(byte servoNum)
{
  byte servoStep = 1;
  if (servoNum == SERVO_CLAW || servoNum == SERVO_HAND_R)
  {
    servoStep = 2;
  }
  
  moveServo(servoNum, servoPosition[servoNum] + servoStep); 
}

void moveServoDown(byte servoNum)
{
  byte servoStep = 1;
  if (servoNum == SERVO_CLAW || servoNum == SERVO_HAND_R)
  {
    servoStep = 2;
  }
    
  moveServo(servoNum, servoPosition[servoNum] - servoStep);   
}

void moveServo(byte servoNum, byte percent)
{
  int max = 100;
  int min = 2;
  if (servoNum == SERVO_CLAW) //servoClaw = 42 - 78%
  {
    min = 42;
    max = 78;
  }
  if (servoNum == SERVO_HAND_M) //servoHandM 10 - 90%
  {
    min = 10;
    max = 90;
  }
  if (servoNum == SERVO_SHLD_M) //servoShldM 30 - 80%
  {
    min = 30;
    max = 76;
  }
  
  if (percent > max)
  {
    percent = max;
  }
  else if (percent < min)
  {
    percent = min;
  }
  else {
    servoActive = 1;
    moveServoTo(servoNum, percent);
  }
}

void moveServoTo(byte servoNum, byte percent)
{
  Serial.print("Servo: "); Serial.print(servoNum); Serial.print(" % "); Serial.println(percent);
  
  servoPosition[servoNum] = percent;

  int servoPos = (int) ( (float) ( SERVO_MAX - SERVO_MIN ) / 100 * percent) + SERVO_MIN;
  //byte servoDelay = 1;
  
  pwm.setPWM(servoNum, 0, servoPos );
}

void readServoSettings()
{
  ServoSetting servoSet;
  EEPROM.get(eeAddress, servoSet);
  
  //Serial.print("Claw: "); Serial.println(servoSet.claw);
  
  if (servoSet.claw < 101 && servoSet.claw > 0)
  {
    servoPosition[SERVO_CLAW] = servoSet.claw;
  }
  
  if (servoSet.handr < 101 && servoSet.handr > 0)
  {
    servoPosition[SERVO_HAND_R] = servoSet.handr;
  }
  
  if (servoSet.handm < 101 && servoSet.handm > 0)
  {
    servoPosition[SERVO_HAND_M] = servoSet.handm;
  }
  
  if (servoSet.elbow < 101 && servoSet.elbow > 0)
  {
    servoPosition[SERVO_ELBOW] = servoSet.elbow;
  }
  
  if (servoSet.shldm < 101 && servoSet.shldm > 0)
  {
    servoPosition[SERVO_SHLD_M] = servoSet.shldm;
  }
  
  if (servoSet.shldr < 101 && servoSet.shldr > 0)
  {
    servoPosition[SERVO_SHLD_R] = servoSet.shldr;
  }
}


void writeServoSettings()
{
//int eeAddress = sizeof(float)
  
  ServoSetting servoSet;
  
  servoSet.claw  = servoPosition[SERVO_CLAW];
  servoSet.handr = servoPosition[SERVO_HAND_R];
  servoSet.handm = servoPosition[SERVO_HAND_M];
  servoSet.elbow = servoPosition[SERVO_ELBOW];
  servoSet.shldm = servoPosition[SERVO_SHLD_M];
  servoSet.shldr = servoPosition[SERVO_SHLD_R];

  EEPROM.put(eeAddress, servoSet);
}
