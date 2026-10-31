#include <math.h>

//Hardware setup
int32_t pulseSupply[] = {0, 0, 0};
double PeriodOfPulse = 1000;
uint16_t moveDelay = 1000;       // pulse delay (μs) — increase for slower motion
const uint16_t SLOW_DELAY = 4000; // used during descent to pick position
uint8_t lmPin[] = { 15, 17, 19 };    
uint8_t dirPin[] = { 8, 10, 12 };    
uint8_t pulsePin[] = { 9, 11, 13 };  
uint8_t psetdir[] = { 0, 0, 1 }; 
uint8_t hsetdir[] = { 1, 1, 0 };

// ===== Electromagnet =====
const uint8_t MAG_PIN = 2;   // uses D2

int32_t pulseHome[] = { 300, 900, -300 };

bool flagHome = false;

double angleSP[] = { 0, 0, 0 };
int32_t pulseSP[] = { 0, 0, 0 };
double positionSP[] = { 0, 0, 0 };
double homeAngle[] = {0, 90, -90};
double angleNow[] = { 0, 0, 0 };
int32_t pulseNow[] = { 0, 0, 0 };
double positionNow[] = { 0, 0, 0 };

bool stringCompleted = false;

const double L1 = 140, L2 = 140, L3 = 140, L4 = 54, L5 = 0;
String inputString;

volatile bool isBusy = false;

double dropPos[3] = {106.17, 227.69, 122.74};

// Prototype
void splitDataKinematic(String inputString);
void forward_kinematics(double angle_input[]);
bool inverse_kinematics1(double pos_input[]);
void Sethome(uint8_t NoJoint, int32_t pulse, uint16_t timeDelay);
uint32_t ration_convert(uint8_t NoJoint);
void run_stepper(double angle_input[]);
void run_synchronous_stepper(double angle_input[]);
void generate_pulse(uint8_t NoJoint, int32_t pulseNumber, uint16_t timeDelay);

// Pick-and-place using the magnet
void pick_and_place(double pick_pos[3]);

void setup() { 
  Serial.begin(115200);
  inputString.reserve(200);

  for (uint8_t i = 0; i < 3; i++) {
    pinMode(lmPin[i], INPUT_PULLUP);
    pinMode(dirPin[i], OUTPUT);
    pinMode(pulsePin[i], OUTPUT);
    digitalWrite(dirPin[i], 0);
    digitalWrite(pulsePin[i], 0);
  }

  pinMode(MAG_PIN, OUTPUT);
  digitalWrite(MAG_PIN, LOW);

  Serial.println("[INFO] Arduino ready.");
}

void loop() {

  // ===== ALWAYS read Serial =====
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    inputString += inChar;

    if (inChar == ')') {
      stringCompleted = true;
      break;
    }
  }

  // ===== HANDLE COMMAND =====
  if (stringCompleted) {
    stringCompleted = false;
    inputString.trim();

    char cmd = inputString.charAt(0);

    // ===== COMMAND I =====
    if (cmd == 'I') {
      if (!isBusy) {
        splitDataKinematic(inputString);
        isBusy = true;

        pick_and_place(positionSP);

        Serial.println("Done");
        isBusy = false;
      }
      else {
        Serial.println("[BUSY] Ignore I()");
      }
    }

    // ===== COMMAND F =====
    else if (cmd == 'F') {
      if (!isBusy) {
        splitDataKinematic(inputString);
        forward_kinematics(angleSP);
        run_synchronous_stepper(angleSP);

        Serial.println("Done");
      }
      else {
        Serial.println("[BUSY] Ignore F()");
      }
    }

    // ===== COMMAND H (HIGH PRIORITY) =====
    else if (cmd == 'H') {
      Serial.println("[CMD] HOME");

      // homing is still allowed while busy
      isBusy = true;

      for (uint8_t i = 0; i < 3; i++) {
        Sethome(i, pulseHome[i], 2000);
        angleNow[i] = homeAngle[i];
        pulseNow[i] = 0;
      }

      Serial.println("Done");
      isBusy = false;
    }

    inputString = "";
  }
}
/* Split data */
void splitDataKinematic(String inputString) {
  uint8_t charStart = inputString.indexOf('(');
  uint8_t charEnd = inputString.indexOf(')');
  String data_raw = inputString.substring(charStart + 1, charEnd);
  data_raw.trim();

  uint8_t charSpace[2];
  uint8_t q = 0;
  for (uint8_t p = 0; p < data_raw.length(); p++) {
    if (data_raw[p] == ' ') {
      charSpace[q++] = p;
      if (q == 2) break;
    }
  }

  if (inputString[0] == 'I') {
    positionSP[0] = data_raw.substring(0, charSpace[0]).toDouble();
    positionSP[1] = data_raw.substring(charSpace[0] + 1, charSpace[1]).toDouble();
    positionSP[2] = data_raw.substring(charSpace[1] + 1).toDouble();
  }

  if (inputString[0] == 'F') {
    angleSP[0] = data_raw.substring(0, charSpace[0]).toDouble();
    angleSP[1] = data_raw.substring(charSpace[0] + 1, charSpace[1]).toDouble();
    angleSP[2] = data_raw.substring(charSpace[1] + 1).toDouble();
  }
}

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

  double theta1 = atan2(Py, Px);
  double r = sqrt(Px*Px + Py*Py);
  double rw = r - (L4 + L5);  
  double s  = Pz - L1;  
  double D = (rw*rw + s*s - L2*L2 - L3*L3) / (2*L2*L3);

  if (D < -1.0 || D > 1.0) {
    return false;
  }
  if (D > 1) D = 1;
  if (D < -1) D = -1;

  double s3 = -sqrt(1 - D*D);
  double c3 = D;
  double theta3 = atan2(s3, c3);

  double theta2 = atan2(s, rw) - atan2(L3*s3, L2 + L3*c3);

  double q1 = theta1 * 180.0 / M_PI;
  double q2 = theta2 * 180.0 / M_PI;
  double q3 = theta3 * 180.0 / M_PI;

  angleSP[0] = q1;
  angleSP[1] = q2;
  angleSP[2] = q3;

  return true;
}

/* Sethome */
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
      delay(5);  //wait, then re-check the limit switch (debounce)
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
}

void run_synchronous_stepper(double angle_input[]) {
  int32_t pulseStart[] = { 0, 0, 0 };
  int32_t pulseOld[] = { 0, 0, 0 };
  pulseSP[0] = ((angle_input[0] - homeAngle[0]) * ration_convert(0)) / 360.0;
  pulseSP[1] = ((angle_input[1] - homeAngle[1]) * ration_convert(1)) / 360.0;
  double q23_now = angleNow[1] + angleNow[2];
  double q23_sp  = angle_input[1] + angle_input[2];
  pulseSP[2] = ((q23_sp - (homeAngle[1] + homeAngle[2])) * ration_convert(2)) / 360.0;
  for (uint8_t p = 0; p < 3; p++) {
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
    }

    for (uint8_t i = 0; i < 3; i++) {
      generate_pulse(i, (pulseNow[i] - pulseOld[i]), moveDelay);
      pulseOld[i] = pulseNow[i];
    }
    time = time + 0.001;
    if (time >= timeRun) break;
  }

  for (uint8_t i = 0; i < 3; i++) {
    generate_pulse(i, (pulseSP[i] - pulseNow[i]), moveDelay);
    pulseNow[i] = pulseSP[i];
    angleNow[i] = angleSP[i];
    positionNow[i] = positionSP[i];
  }
  Serial.println("--------------------------------------------------------------------");
}

void generate_pulse(uint8_t NoJoint, int32_t pulseNumber, uint16_t timeDelay) { 
  if (pulseNumber > 0)  
  { 
    if(NoJoint == 0 || NoJoint == 2) digitalWrite(dirPin[NoJoint], 1); else digitalWrite(dirPin[NoJoint], 0);
  } 
  else { 
    if(NoJoint == 0 || NoJoint == 2) digitalWrite(dirPin[NoJoint], 0); else digitalWrite(dirPin[NoJoint], 1); 
  } 
 
  for (uint32_t k = 0; k < abs(pulseNumber); k++) { 
    digitalWrite(pulsePin[NoJoint], 1); 
    if (NoJoint == 1) delayMicroseconds(timeDelay*1.2); 
    else delayMicroseconds(timeDelay); 
    digitalWrite(pulsePin[NoJoint], 0); 
    if (NoJoint == 1) delayMicroseconds(timeDelay*1.2); 
    else delayMicroseconds(timeDelay); 
  } 
} 

void pick_and_place(double pick_pos[3]) {
  Serial.println("[TASK] Pick & Place start");

  // 1) Approach from above (Z + 4mm) at normal speed
  double approach[3] = { pick_pos[0], pick_pos[1], pick_pos[2] + 4.0 };
  inverse_kinematics1(approach);
  moveDelay = 1000;
  run_synchronous_stepper(angleSP);
  delay(200);

  // 2) Descend slowly onto the object (Z = 0)
  inverse_kinematics1(pick_pos);
  moveDelay = SLOW_DELAY;
  run_synchronous_stepper(angleSP);
  moveDelay = 1000;
  delay(200);

  // 3) Turn magnet on -> pick up the object
  digitalWrite(MAG_PIN, HIGH);
  delay(400);

  // 4) Lift the object
  inverse_kinematics1(approach);
  run_synchronous_stepper(angleSP);
  delay(200);

  // 5) Move to the drop position
  double place_up[3] = { dropPos[0], dropPos[1], dropPos[2] + 40.0};
  inverse_kinematics1(place_up);
  run_synchronous_stepper(angleSP);
  delay(200);

  // 6) Lower to release
  inverse_kinematics1(dropPos);
  run_synchronous_stepper(angleSP);
  delay(200);

  // 7) Turn magnet off to release the object
  digitalWrite(MAG_PIN, LOW);
  delay(400);

  // 8) Lift back up
  inverse_kinematics1(place_up);
  run_synchronous_stepper(angleSP);
  delay(200);

  Serial.println("[TASK] Pick & Place Finished");
}