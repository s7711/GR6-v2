// This code runs the motor for the GR6
// Version 250630: SV commands to set the motor speed
// Version 250708: Added water pump
// Version 250710: PI controller
// Version 250717: Separate left and right constants
// Version 250814: Changed default PI constants
// Version 250815: Added D term (for PID controller); Separate Min Max intergral terms
// Version 250816: Trying to remove glitches. PID tuned.
// Version 250819: Adding ultrasonic sensors;
// Version 260720: Fixed EN (encoder position) telemetry — it was going
//   through sendFloat(), which multiplies by 100 and truncates to a
//   16-bit int for transmission. That's fine for velocities (small
//   values) but silently wraps encoder position after only ~327 counts
//   of travel (about 1.3m at ~250 counts/metre) even though the
//   firmware's own internal `long LM_position`/`RM_position` never
//   wrapped — only the wire value did. Now sent raw via a dedicated
//   sendLong(), matching what the Pi side already expected.

#define VERSION "260720#1.GR6"

// The motors are driven by an L298 module
// The feedback is via 2x A3144 hall effect sensors on each motor (4 total)
// The A3144 require pull-up resistors in the Arduino

// Pins for the left motor
#define LM_ENA    10   // Motor speed PWM
#define LM_IN1     6   // Direction control
#define LM_IN2     7  
#define LM_ENC_A   3   // Quadrature encoder A (interrupt pin)
#define LM_ENC_B   5   // Quadrature encoder B

// Pins for the right motor
#define RM_ENA    11   // Motor speed PWM
#define RM_IN1     8   // Direction control
#define RM_IN2     9  
#define RM_ENC_A   2   // Quadrature encoder A (interrupt pin)
#define RM_ENC_B   4   // Quadrature encoder B

int ultrasonicPins[] = {A5, A4, A3, A2, A1};

volatile long LM_position = 0L;   // Updated in the ISR, motor's position
volatile long RM_position = 0L;

// Pins for the pump
#define PUMP_ENA  12
int wp_state = LOW;

// Control loop constants
unsigned long lastCtrlTime = 0;               // for fixed control loop timestep
unsigned int CtrlLoopStep = 100;              // millis
const float dt = (float) CtrlLoopStep/1000.0; // seconds
const float cf = 1.0 / dt;                    // control loop frequency, 1/seconds

float LM_Kp = 1.0,        RM_Kp = 1.0;     // Proportional gain
float LM_Ki = 3.0,        RM_Ki = 3.0;     // Integral gain
float LM_Kd = 0.0,        RM_Kd = 0.0;     // Differential gain
float LM_Kf = 0.0,        RM_Kf = 0.0;     // Feedforward term - see motor comments below
float LM_Ka = 0.9,        RM_Ka = 0.9;     // Differential error filter
float LM_Kb = 0.6,        RM_Kb = 0.6;     // Velocity filter
float LM_Mi = 150.0,      RM_Mi = 150.0;   // Maximum integral error
float LM_Mj = -50.0,      RM_Mj = -50.0;   // Minimum integral error
float LM_Id = 0.97,       RM_Id = 0.97;    // Integral error decay
int   LM_Db = 5,          RM_Db = 5;       // Deadband
float LM_Am = 100.0*dt,   RM_Am = 100.0*dt; // Maximum set point change per cycle

// Control‐loop state
long  prevLM_pos = 0,    prevRM_pos = 0;
float filtLM_vel = 0.0,  filtRM_vel = 0.0;
float prev_errLM = 0.0,  prev_errRM = 0.0;
float filtLM_dErr = 0.0, filtRM_dErr = 0.0;
float integralLM = 0.0,  integralRM = 0.0;
float errLM = 0.0,       errRM = 0.0;
float outLMf = 0.0,      outRMf = 0.0;
int   outLM = 0,         outRM = 0;
int ctrlEnabled = 0;  // Ramps from 1 to 4 when control starts, to avoid glitches

// Motor output limit
float MotorMax = 250.0;
float MotorMin = -MotorMax;

// Buffer to receive user's commands
#define MAX_CMD_LEN (80)
char rxCmd[MAX_CMD_LEN+1] = ""; // For commands received
int rxCmdLen = 0;               // Length, to avoid finding the end of the string each time

// Variables for updating the user
unsigned long lastUpdateTime = 0L;

// If no serial commands are received for a while then stop
const unsigned long CMD_TIMEOUT = 2000;  // ms
unsigned long TimeSpeedLastCommand = 0L;
unsigned long TimePumpLastCommand = 0L;

// Values from the RPI
// Will be set in user_command()
float LM_target_setvel = 0.0;
float RM_target_setvel = 0.0;
float LM_setvel = 0;      // Input to the control algorithm, after Amax applied
float RM_setvel = 0;

// Updates to the RPI sent one at a time
// Counts the updates
int whichUpdate = 1;
int whichControlUpdate = 1;


// Interrupt service routines for the encoders
// Triggered when ENC_A changes
// The ++ and -- depend on how the wires are connected
// and the orientation of the motors in the vehicle
void LM_encoderISR()
{
    if (digitalRead(LM_ENC_A) == digitalRead(LM_ENC_B))
        LM_position--;
    else
        LM_position++;
}

void RM_encoderISR()
{
    if (digitalRead(RM_ENC_A) == digitalRead(RM_ENC_B))
        RM_position++;
    else
        RM_position--;
}


// Functions to set the motor speed
// Measurement of the motor with no load showed:
// Input Output (counts/second)
//    0      0
//   10      0
//   20      8
//   30     40
//   40     80
//   50    104
//   60    124
//   70    136
//   80    150
//   90    160
//  100    170
// ...
//  150    180
//  200    188
//  250    192
// See "Motor input vs. output.xlsx
//
// From this we can see that
// * Below 20 the motor stalls
// * There's a linear region up to about 50
// * A second linear region up to about 100
// * Then the speed increase drops off rapidly
void LM_setMotorSpeed(int motorSpeed)
{
  int absMotorSpeed = abs(motorSpeed);
  if( absMotorSpeed <= LM_Db ) absMotorSpeed = 0;          // Deadband
  else if( absMotorSpeed < 30 ) absMotorSpeed += 30;       // Overcome stiction
  if( absMotorSpeed > MotorMax ) absMotorSpeed = MotorMax; // Limit output

  if( motorSpeed > 0)
  {
    digitalWrite(LM_IN1, HIGH);
    digitalWrite(LM_IN2, LOW);
  }
  else
  {
    digitalWrite(LM_IN1, LOW);
    digitalWrite(LM_IN2, HIGH);
  }
  
  analogWrite(LM_ENA, absMotorSpeed);
}

void RM_setMotorSpeed(int motorSpeed)
{
  int absMotorSpeed = abs(motorSpeed);
  if( absMotorSpeed <= LM_Db ) absMotorSpeed = 0;          // Deadband
  else if( absMotorSpeed < 30 ) absMotorSpeed += 30;       // Overcome stiction
  if( absMotorSpeed > MotorMax ) absMotorSpeed = MotorMax; // Limit output

  if( motorSpeed > 0)
  {
    digitalWrite(RM_IN1, HIGH);
    digitalWrite(RM_IN2, LOW);
  }
  else
  {
    digitalWrite(RM_IN1, LOW);
    digitalWrite(RM_IN2, HIGH);
  }
  
  analogWrite(RM_ENA, absMotorSpeed);
}


void user_command(unsigned long now)
{
  // Interprets the user commands in rxBuf - here to keep the loop looking tidy
    int l1, l2;
    // Interpret the command
    // SV - set the target velocity
    if( sscanf(rxCmd, "SV %d %d", &l1, &l2) == 2)
    {      
      LM_target_setvel = (float) constrain(l1,-200,200); 
      RM_target_setvel = (float) constrain(l2,-200,200);

      if( l1 == 0 && l2 == 0)
      {
        ctrlEnabled = 0; // Stop control: special situation to coast to zero speed
        LM_target_setvel = 0.0;
        RM_target_setvel = 0.0;
        LM_setMotorSpeed(0);
        RM_setMotorSpeed(0);
      }
      else if( ctrlEnabled == 0 )
        ctrlEnabled = 1; // Enable control, using a ramp (see PID loop)
      TimeSpeedLastCommand = now;
    }
    // Set the water pump
    else if( sscanf(rxCmd, "WP %d", &l1) == 1)
    {
      wp_state = l1 ? HIGH : LOW;
      digitalWrite(PUMP_ENA, wp_state);
      TimePumpLastCommand = now;
    }
    // Set the control parameters
    // No checks! Take good care
    else if( sscanf(rxCmd, "Kp %d %d", &l1, &l2) == 2) LM_Kp = l1 * 0.01,    RM_Kp = l2 * 0.01;
    else if( sscanf(rxCmd, "Ki %d %d", &l1, &l2) == 2) LM_Ki = l1 * 0.01,    RM_Ki = l2 * 0.01;
    else if( sscanf(rxCmd, "Kd %d %d", &l1, &l2) == 2) LM_Kd = l1 * 0.01,    RM_Kd = l2 * 0.01;
    else if( sscanf(rxCmd, "Kf %d %d", &l1, &l2) == 2) LM_Kf = l1 * 0.01,    RM_Kf = l2 * 0.01;
    else if( sscanf(rxCmd, "Ka %d %d", &l1, &l2) == 2) LM_Ka = l1 * 0.01,    RM_Ka = l2 * 0.01;
    else if( sscanf(rxCmd, "Kb %d %d", &l1, &l2) == 2) LM_Kb = l1 * 0.01,    RM_Kb = l2 * 0.01;
    else if( sscanf(rxCmd, "Db %d %d", &l1, &l2) == 2) LM_Db = l1,           RM_Db = l2;
    else if( sscanf(rxCmd, "Mi %d %d", &l1, &l2) == 2) LM_Mi = (float)l1,    RM_Mi = (float)l2;
    else if( sscanf(rxCmd, "Mj %d %d", &l1, &l2) == 2) LM_Mj = (float)l1,    RM_Mj = (float)l2;
    else if( sscanf(rxCmd, "Id %d %d", &l1, &l2) == 2) LM_Id = l1 * 0.01,    RM_Id = l2 * 0.01;
    else if( sscanf(rxCmd, "Am %d %d", &l1, &l2) == 2) LM_Am = (float)l1*dt, RM_Am = (float) l2*dt;

    // Reset the command buffer ready for the next command
    rxCmd[0] = 0;
    rxCmdLen = 0;
}

// Formatting functions to send two values to the RPI
void sendFloat(char *field, float v1, float v2)
{
  Serial.print(field);
  Serial.print(" ");
  Serial.print((int)(v1*100.0));
  Serial.print(" ");
  Serial.println((int)(v2*100.0));
}

void sendInt(char *field, int v1, int v2)
{
  Serial.print(field);
  Serial.print(" ");
  Serial.print(v1);
  Serial.print(" ");
  Serial.println(v2);
}

// For values too large for a 16-bit int (e.g. encoder position) — prints
// the raw long with no scaling and no truncation.
void sendLong(char *field, long v1, long v2)
{
  Serial.print(field);
  Serial.print(" ");
  Serial.print(v1);
  Serial.print(" ");
  Serial.println(v2);
}

void sendUltrasonic(int sensor)
{
  int sensorPin = ultrasonicPins[sensor];
  long d;
  
  // Send trigger pulse
  pinMode(sensorPin, OUTPUT);
  digitalWrite(sensorPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorPin, LOW);

  // Switch to input to read echo
  pinMode(sensorPin, INPUT);
  d = pulseIn(sensorPin, HIGH, 13000);  // Timeout after 13ms just over 2m

  // Calculate distance in mm
  d = (d * 343) / 2000; // *0.343 mm/s two ways
  if( d > 2000 || d == 0) d = -1;

  // Print result
  Serial.print("U");
  Serial.print(sensor);
  Serial.print(" ");
  Serial.println(d);
}


void setup() {
    pinMode(LM_ENA, OUTPUT);
    pinMode(LM_IN1, OUTPUT);
    pinMode(LM_IN2, OUTPUT);
    pinMode(LM_ENC_A, INPUT_PULLUP);
    pinMode(LM_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(LM_ENC_A), LM_encoderISR, CHANGE);
    
    pinMode(RM_ENA, OUTPUT);
    pinMode(RM_IN1, OUTPUT);
    pinMode(RM_IN2, OUTPUT);
    pinMode(RM_ENC_A, INPUT_PULLUP);
    pinMode(RM_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(RM_ENC_A), RM_encoderISR, CHANGE);

    pinMode(PUMP_ENA, OUTPUT);
    wp_state = LOW;
    digitalWrite(PUMP_ENA, wp_state);

    // Ultrasonic sensors
    pinMode(A5, LOW);
    pinMode(A4, LOW);
    pinMode(A3, LOW);
    pinMode(A4, LOW);
    pinMode(A1, LOW);

    Serial.begin(115200);

    lastCtrlTime = millis();
    lastUpdateTime = millis();
}

void loop()
{
  unsigned long now = millis();
  
  // Process serial commands
  int c = Serial.read();
  if( c >= 0 ) // There's a valid character
  {
    rxCmd[rxCmdLen++] = (char) c;
    rxCmd[rxCmdLen] = (char) 0;
    if( rxCmdLen >= MAX_CMD_LEN ) rxCmdLen--; // Limit length
    if( (char)c == '\n') user_command(now);
  }
  
  // Stop if no speed commands are sent
  if(now-TimeSpeedLastCommand > CMD_TIMEOUT)
  {
      LM_target_setvel = 0.0;
      RM_target_setvel = 0.0;
      LM_setMotorSpeed(0);
      RM_setMotorSpeed(0);
      ctrlEnabled = 0;
      TimeSpeedLastCommand = now;
  }
  
  // Turn off pump if no pump commands are sent
  if(now-TimePumpLastCommand > CMD_TIMEOUT)
  {
      wp_state = LOW;
      digitalWrite(PUMP_ENA, wp_state);
      TimePumpLastCommand = now;
  }

  unsigned long timeSinceLastCtrl = now - lastCtrlTime; // Works even with wrap-around
  if(timeSinceLastCtrl >= CtrlLoopStep)
  {
    lastCtrlTime += CtrlLoopStep;

    // Compute setvel from target_setvel using Amax
    float dLM = LM_target_setvel - LM_setvel;
    float dRM = RM_target_setvel - RM_setvel;
    dLM = constrain(dLM, -LM_Am, +LM_Am);
    dRM = constrain(dRM, -RM_Am, +RM_Am);
    LM_setvel += dLM;
    RM_setvel += dRM;

    // Raw velocity
    dLM = (LM_position - prevLM_pos) * cf; // cf = 1/dt
    dRM = (RM_position - prevRM_pos) * cf;
    prevLM_pos = LM_position;
    prevRM_pos = RM_position;
  
    // Filter velocity
    filtLM_vel = LM_Kb * filtLM_vel + (1.0 - LM_Kb) * dLM;  // Kb = 0 would give rawLM_vel straight into the filter
    filtRM_vel = RM_Kb * filtRM_vel + (1.0 - RM_Kb) * dRM;
  
    // Error: based on filtered velocity
    errLM = LM_setvel - filtLM_vel;
    errRM = RM_setvel - filtRM_vel;

    // Integrate error
    integralLM *= LM_Id;
    integralRM *= RM_Id;
    integralLM = constrain(integralLM + errLM*dt, LM_Mj, LM_Mi);
    integralRM = constrain(integralRM + errRM*dt, RM_Mj, RM_Mi);

    // Differential error
    dLM = (errLM - prev_errLM) * cf;
    dRM = (errRM - prev_errRM) * cf;
    filtLM_dErr = filtLM_dErr * LM_Ka + (1.0 - LM_Ka) * dLM;
    filtRM_dErr = filtRM_dErr * RM_Ka + (1.0 - LM_Ka) * dRM;
    prev_errLM = errLM;
    prev_errRM = errRM;
  
    outLMf = LM_Kf*LM_setvel + LM_Kp*errLM + LM_Ki*integralLM + LM_Kd*filtLM_dErr;
    outRMf = RM_Kf*RM_setvel + RM_Kp*errRM + RM_Ki*integralRM + RM_Kd*filtRM_dErr;

    outLM = (int)outLMf;
    outRM = (int)outRMf;
    
    if(ctrlEnabled > 0)
    {
      // ctrlEnabled used to scale motor output at start to avoid glitches
      outLM = ctrlEnabled * outLM / 4;
      outRM = ctrlEnabled * outRM / 4;
      ctrlEnabled = ctrlEnabled >= 4 ? 4 : ctrlEnabled + 1;
      LM_setMotorSpeed(outLM);
      RM_setMotorSpeed(outRM);
    }
    else
    {
      integralLM *= 0.3;
      integralRM *= 0.3;
    }
  }
  
  // Update the RPI: not if within 20ms of a control loop step
  else if( now - lastUpdateTime >= 10 && timeSinceLastCtrl < CtrlLoopStep - 20)
  {
    lastUpdateTime += 10;
    
    switch(whichUpdate)
    {
      case 1: sendLong("EN", LM_position, RM_position); break;
      case 2: sendFloat("SV", LM_setvel, RM_setvel); break;
      case 3: sendFloat("FV", filtLM_vel, filtRM_vel); break;
      case 4: sendFloat("ER", errLM, errRM); break;    
      case 5: sendFloat("EI", integralLM, integralRM); break;
      case 6: sendFloat("ED", filtLM_dErr, filtRM_dErr); break;   
      case 7: sendInt("MO", outLM, outRM);
      case 8: Serial.print("WP "); Serial.println(wp_state); break;
      case 9: Serial.print("GO "); Serial.println(ctrlEnabled); break;
      case 10: sendUltrasonic(0); break;
      case 11: sendUltrasonic(1); break;
      case 12: sendUltrasonic(2); break;
      case 13: sendUltrasonic(3); break;
      case 14: sendUltrasonic(4); break;
      case 15: switch(whichControlUpdate)
        {
        case 1: sendFloat("Kp", LM_Kp, RM_Kp); break;
        case 2: sendFloat("Ki", LM_Ki, RM_Ki); break;
        case 3: sendFloat("Kd", LM_Kd, RM_Kd); break;
        case 4: sendFloat("Kf", LM_Kf, RM_Kf); break;
        case 5: sendFloat("Ka", LM_Ka, RM_Ka); break;
        case 6: sendFloat("Kb", LM_Kb, RM_Kb); break;
        case 7: sendInt("Db", LM_Db, RM_Db); break;
        case 8: sendInt("Mi", (int)LM_Mi, (int)RM_Mi); break;
        case 9: sendInt("Mj", (int)LM_Mj, (int)RM_Mj); break;
        case 10: sendFloat("Id", LM_Id, RM_Id); break;
        case 11: sendInt("Am",(int)(LM_Am*cf), (int)(RM_Am*cf)); break;
        case 12: Serial.print("Version "); Serial.println(VERSION);      
        default:
          whichControlUpdate = 0;
        }
        whichControlUpdate++;
      default:
        whichUpdate = 0;
    }
    whichUpdate++;
  }
}
