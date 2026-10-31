//Hardware setup
#include <math.h>
int32_t pulseSupply[] = {0, 0, 0};
double PeriodOfPulse = 1000;
uint8_t lmPin[] = { 15, 17, 19 };    //Limit switch pins
uint8_t dirPin[] = { 8, 10, 12 };    //Direction pins for motors 1,2,3
uint8_t pulsePin[] = { 9, 11, 13 };  //Pulse pins for motors 1,2,3
uint8_t psetdir[] = { 0, 0, 1 }; // Motor direction when moving toward the home sensor
uint8_t hsetdir[] = { 1, 1, 0 }; // Motor direction after touching the sensor, when returning to home
//Variable declarations
int32_t pulseHome[] = { 300, 900, 300 };  //Number of pulses needed to rotate the robot to the desired pose

bool flagHome = false;
double angleSP[] = { 0, 0, 0 };  // joint angle setpoints
int32_t pulseSP[] = { 0, 0, 0 };
double positionSP[] = { 0, 0, 0 };  //robot position

double angleNow[] = { 0, 0, 0 };  // current joint angles
int32_t pulseNow[] = { 0, 0, 0 };
double positionNow[] = { 0, 0, 0 };  //robot position
double homeAngle[] = {0, 90, -90};
bool stringCompleted = false;  //flag set when a complete UART string has been received
const double L1 = 140, L2 = 140, L3 = 140, L4 = 54, L5 = 0.8; // Robot link dimensions taken from SolidWorks
String inputString;  //string holding the received command

void setup() {
  Serial.begin(115200);      //set up UART
  inputString.reserve(200);  //reserve a 200-byte buffer for UART data
  //Configure input/output pins and the initial state of the outputs
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(lmPin[i], INPUT_PULLUP);
    pinMode(dirPin[i], OUTPUT);
    pinMode(pulsePin[i], OUTPUT);

    digitalWrite(dirPin[i], 0);
    digitalWrite(pulsePin[i], 0);
  }
}

void loop() {
  //Read from the UART buffer whenever data is available, until the terminating ")"
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();  // Read the incoming byte into "inChar"
    inputString += inChar;              // Append inChar to "inputString"
    if (inChar == ')')                  // If the terminating character is found
    {
      Serial.print(inputString);
      Serial.print(" ");
      stringCompleted = true;  // set the UART flag to true and stop appending to "inputString"
      break;
    }
    //start processing the received string
    if (stringCompleted == true) {
      stringCompleted = false;
      splitDataKinematic(inputString);  // parse the string with this function
      if (inputString[0] == 'H') {      //a leading H means: go home
        Serial.println("Set home");
        for (uint8_t i = 0; i < 3; i++) {
          Sethome(i, pulseHome[i], 2000);
          angleNow[i] = homeAngle[i];
          pulseNow[i] = (homeAngle[i] * ration_convert(i)) / 360.0;
        }
        
      } else {
        if (inputString[0] == 'F') {  //a leading F means: run forward kinematics
          forward_kinematics(angleSP);
          Serial.print("Angle FK: ");
          for (uint8_t j = 0; j < 3; j++) {
            Serial.print(angleSP[j]);
            Serial.print(" ");
          }
          Serial.print(" | ");

          Serial.print("Position FK: ");
          for (uint8_t j = 0; j < 3; j++) {
            Serial.print(positionSP[j]);
            Serial.print(" ");
          }
          Serial.println(" | ");
          // run_stepper(angleSP);
          run_synchronous_stepper(angleSP);

        } else {
          if (inputString[0] == 'I') {  //a leading I means: run inverse kinematics
            inverse_kinematics1(positionSP);
            Serial.print("Pos IK: ");
            for (uint8_t j = 0; j < 3; j++) {
              Serial.print(positionSP[j]);
              Serial.print(" ");
            }
            Serial.print(" | ");

            Serial.print("Angle FK: ");
            for (uint8_t j = 0; j < 3; j++) {
              Serial.print(angleSP[j]);
              Serial.print(" ");
            }
            Serial.println(" | ");
            // run_stepper(angleSP);
            run_synchronous_stepper(angleSP);

          } else {
            Serial.print(inputString);
            Serial.print(" is a ");  //none of the cases above: invalid command
            Serial.println("Incorrect command!");
          }
        }
      }
      inputString = "";
    }  

  }
}

void splitDataKinematic(String inputString) {
  //Find the positions of ( and ) in the string
  uint8_t charStart = inputString.indexOf('(');
  uint8_t charEnd = inputString.indexOf(')');
  //Strip ( and ) so data_raw holds only numbers and spaces
  String data_raw = inputString.substring(charStart + 1, charEnd);

  //Find the positions of the spaces in data_raw
  uint8_t charSpace[] = { 0, 0 };
  uint8_t q = 0;
  for (uint8_t p = 0; p < data_raw.length(); p++) {
    if (data_raw[p] == ' ') {
      charSpace[q] = p;
      q++;
    }
  }

  //For an I command, parse the data into positionSP
  if (inputString[0] == 'I') {
    positionSP[0] = (data_raw.substring(0, charSpace[0])).toDouble();
    positionSP[1] = (data_raw.substring(charSpace[0] + 1, charSpace[1])).toDouble();
    positionSP[2] = (data_raw.substring(charSpace[1] + 1, data_raw.length())).toDouble();
  }

  //For an F command, parse the data into angleSP
  if (inputString[0] == 'F') {
    angleSP[0] = (data_raw.substring(0, charSpace[0])).toDouble();
    angleSP[1] = (data_raw.substring(charSpace[0] + 1, charSpace[1])).toDouble();
    angleSP[2] = (data_raw.substring(charSpace[1] + 1, data_raw.length())).toDouble();
  }
};

void forward_kinematics(double angle_input[]) {
  double q1 = angle_input[0] * M_PI / 180.0;
  double q2 = angle_input[1] * M_PI / 180.0;
  double q3 = angle_input[2] * M_PI / 180.0;

  positionSP[0] = cos(q1)*(L4 + L5 + L3*cos(q2 + q3) + L2*cos(q2));
  positionSP[1] = sin(q1)*(L4 + L5 + L3*cos(q2 + q3) + L2*cos(q2));
  positionSP[2] = L1 + L3*sin(q2 + q3) + L2*sin(q2);
}
bool inverse_kinematics1(double pos_input[]) {
  double Px = pos_input[0];
  double Py = pos_input[1];
  double Pz = pos_input[2];

  // ===== B1: q1 =====
  double theta1 = atan2(Py, Px);

  // ===== Step 2: coordinate conversion =====
  double r = sqrt(Px*Px + Py*Py);
  double rw = r - (L4 + L5);   // offset ngang
  double s  = Pz - L1;         // vertical offset

  // ===== Step 3: workspace check =====
  double D = (rw*rw + s*s - L2*L2 - L3*L3) / (2*L2*L3);

  if (D < -1.0 || D > 1.0) {
    return false; // outside the workspace
  }
  if (D > 1) D = 1;
  if (D < -1) D = -1;
  // ===== B4: q3 (elbow-down) =====
  double s3 = -sqrt(1 - D*D);   // always negative -> elbow-down
  double c3 = D;
  double theta3 = atan2(s3, c3);

  // ===== B5: q2 =====
  double theta2 = atan2(s, rw) - atan2(L3*s3, L2 + L3*c3);

  // ===== Step 6: convert to degrees =====
  double q1 = theta1 * 180.0 / M_PI;
  double q2 = theta2 * 180.0 / M_PI;
  double q3 = theta3 * 180.0 / M_PI;

  // ===== Step 7: offsets for the robot mechanism =====
  angleSP[0] = q1;
  angleSP[1] = q2;   // depends on your mechanism
  angleSP[2] = q3;

  return true;
}
void Sethome(uint8_t NoJoint,int32_t pulse, uint16_t timeDelay) {
  //Set rotation direction to 0 and keep rotating until the limit switch changes state
  digitalWrite(dirPin[NoJoint], psetdir[NoJoint]);
  while (flagHome == false) {
    digitalWrite(pulsePin[NoJoint], 1);
    if (NoJoint == 1) {
      if (digitalRead(lmPin[NoJoint+1]) == 0) {
        digitalWrite(pulsePin[NoJoint+1], 0);
      }
      else {
      digitalWrite(dirPin[NoJoint+1], psetdir[NoJoint+1]);
      digitalWrite(pulsePin[NoJoint+1], 1);
      }
    }
    delayMicroseconds(timeDelay);
    digitalWrite(pulsePin[NoJoint], 0);
    if (NoJoint == 1) {
      digitalWrite(pulsePin[NoJoint+1], 0);
    }
    delayMicroseconds(timeDelay);
    if (digitalRead(lmPin[NoJoint]) == 0) {
      delayMicroseconds(400);  //wait, then re-check the limit switch (debounce)
      if (digitalRead(lmPin[NoJoint]) == 0) {
        delay(1000);
        flagHome = true;
        break;
      }
    }
  }

  //set the opposite direction and send the pulses configured above
  digitalWrite(dirPin[NoJoint], hsetdir[NoJoint]);
  for (uint32_t p = 0; p < abs(pulse); p++) {
    digitalWrite(pulsePin[NoJoint], 1);
    if (NoJoint == 1) delayMicroseconds(timeDelay);
    else delayMicroseconds(timeDelay);
    digitalWrite(pulsePin[NoJoint], 0);
    if (NoJoint == 1) delayMicroseconds(timeDelay);
    else delayMicroseconds(timeDelay);
  }
  flagHome = false;
  delay(1000);
}

uint32_t ration_convert(uint8_t NoJoint) {
  if (NoJoint == 0) return 7200;
  if (NoJoint == 1) return 7200;
  if (NoJoint == 2) return 7200;
  return 7200; 
}

void run_stepper(double angle_input[]) {
  int32_t pulseSupply[] = { 0, 0, 0 };
  for (uint8_t p = 0; p < 3; p++) {
    pulseSP[p] = ((angle_input[p] - homeAngle[p]) * ration_convert(p)) / 360.0;
    pulseSupply[p] = pulseSP[p] - pulseNow[p];

    if (pulseSupply[p] > 0) {
      if (p == 0) digitalWrite(dirPin[p], 1);
      else digitalWrite(dirPin[p], 0);
      //if (p == 1) digitalWrite(dirPin[p], 0); else digitalWrite(dirPin[p], 1);
    } else {
      if (p == 0) digitalWrite(dirPin[p], 0);
      else digitalWrite(dirPin[p], 1);
      //if (p == 1) digitalWrite(dirPin[p], 0); else digitalWrite(dirPin[p], 1);
    }

    for (uint32_t k = 0; k < abs(pulseSupply[p]); k++) {
      digitalWrite(pulsePin[p], 1);
      if (p == 1) delayMicroseconds(800);
      else delayMicroseconds(1000);
      digitalWrite(pulsePin[p], 0);
      if (p == 1) delayMicroseconds(800);
      else delayMicroseconds(1000);
    }

    angleNow[p] = angleSP[p];
    positionNow[p] = positionSP[p];
    pulseNow[p] = pulseSP[p];
  }

  Serial.print("PulseSP: ");
  for (uint8_t p = 0; p < 3; p++) {
    Serial.print(pulseSP[p]);
    Serial.print(" ");
  }
  Serial.print(" | ");

  Serial.print("PulseSupply: ");
  for (uint8_t p = 0; p < 3; p++) {
    Serial.print(pulseSupply[p]);
    Serial.print(" ");
  }
  Serial.print(" | ");

  Serial.print("PulseNow: ");
  for (uint8_t p = 0; p < 3; p++) {
    Serial.print(pulseNow[p]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("------------------------------------------------------------------------");
}
void run_synchronous_stepper(double angle_input[]) {
  int32_t pulseStart[] = { 0, 0, 0 };
  int32_t pulseOld[] = { 0, 0, 0 };
  for (uint8_t p = 0; p < 3; p++) {
    pulseSP[p] = ((angle_input[p]) * ration_convert(p)) / 360.0;
    pulseSupply[p] = pulseSP[p] - pulseNow[p];
    pulseStart[p] = pulseNow[p];
    pulseOld[p] = pulseNow[p];
  }

  double time = 0.00;
  double timeRun = max(max(abs(pulseSupply[0]), abs(pulseSupply[1])), abs(pulseSupply[2])) * (PeriodOfPulse * 1E-6);

  Serial.print("Time Run: ");
  Serial.print(timeRun);
  Serial.print("  | ");

  Serial.print("PulseSP: ");
  for (uint8_t p = 0; p < 3; p++) {
    Serial.print(pulseSP[p]);
    Serial.print(" ");
  }
  Serial.print(" | ");

  Serial.print("PulseNow: ");
  for (uint8_t p = 0; p < 3; p++) {
    Serial.print(pulseNow[p]);
    Serial.print(" ");
  }
  Serial.print(" | ");

  Serial.print("PulseSupply: ");
  for (uint8_t p = 0; p < 3; p++) {
    Serial.print(pulseSupply[p]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("------------------------------------------------------------------------");

  while (time < timeRun) {
    for (uint8_t i = 0; i < 3; i++) {
      pulseNow[i] = pulseStart[i] + 3 * ((pulseSP[i] - pulseStart[i])) * time * time / (timeRun * timeRun) - 2 * (pulseSP[i] - pulseStart[i]) * time * time * time / (timeRun * timeRun * timeRun);
      // Serial.print(pulseNow[i]);
      // Serial.print(" |  ");
    }

    for (uint8_t i = 0; i < 3; i++) {
      generate_pulse(i, (pulseNow[i] - pulseOld[i]), 1000);
      pulseOld[i] = pulseNow[i];
    }
    time = time + 0.001;
    // Serial.println(time);
    if (time >= timeRun) break;
  }

  for (uint8_t i = 0; i < 3; i++) {
    generate_pulse(i, (pulseSP[i] - pulseNow[i]),1000);
    pulseNow[i] = pulseSP[i];
    angleNow[i] = angleSP[i];
    positionNow[i] = positionSP[i];
  }
  Serial.println("--------------------------------------------------------------------");
}
void generate_pulse(uint8_t NoJoint, int32_t pulseNumber, uint16_t timeDelay) { 
  if (pulseNumber > 0)  
  { 
    if(NoJoint == 0) digitalWrite(dirPin[NoJoint], 1); else digitalWrite(dirPin[NoJoint], 0); 
  } 
  else { 
    if(NoJoint == 0) digitalWrite(dirPin[NoJoint], 0); else digitalWrite(dirPin[NoJoint], 1); 
  } 
 
  for (uint32_t k = 0; k < abs(pulseNumber); k++) { 
    digitalWrite(pulsePin[NoJoint], 1); 
    if (NoJoint == 1) delayMicroseconds(timeDelay); 
    else delayMicroseconds(timeDelay); 
    digitalWrite(pulsePin[NoJoint], 0); 
    if (NoJoint == 1) delayMicroseconds(timeDelay); 
    else delayMicroseconds(timeDelay); 
  } 
} 

