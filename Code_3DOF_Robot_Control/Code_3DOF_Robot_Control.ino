//Thiết lập phần cứng
#include <math.h>
int32_t pulseSupply[] = {0, 0, 0};
double PeriodOfPulse = 1000;
uint8_t lmPin[] = { 15, 17, 19 };    //Chân công tắc hành trình
uint8_t dirPin[] = { 8, 10, 12 };    //Chân điều khiển hướng động cơ 1,2,3
uint8_t pulsePin[] = { 9, 11, 13 };  //Chân phát xung điều khiển động cơ 1,2,3
uint8_t psetdir[] = { 0, 0, 1 }; // Biến định nghĩa chiều quay của động cơ khi quay về vị trí cảm biến home
uint8_t hsetdir[] = { 1, 1, 0 }; // Biến định nghĩa chiều quay của động cơ khi chạm cảm biến và quay về vị trí home 
//Khai báo biến
int32_t pulseHome[] = { 300, 900, 300 };  //Khai báo số xung cần phát để robot xoay đến tư thế mong muốn

bool flagHome = false;
double angleSP[] = { 0, 0, 0 };  // khai báo biến chứa vị trí góc của khớp
int32_t pulseSP[] = { 0, 0, 0 };
double positionSP[] = { 0, 0, 0 };  //khai báo biến chứa vị trí của robot

double angleNow[] = { 0, 0, 0 };  // khai báo biến chưa vị trí góc của khớp
int32_t pulseNow[] = { 0, 0, 0 };
double positionNow[] = { 0, 0, 0 };  //khai báo biến chứa vị trí của robot
double homeAngle[] = {0, 90, -90};
bool stringCompleted = false;  //khai báo cờ nhận biết khi nào thì kết thúc nhận chuỗi dữ liệu UART
const double L1 = 140, L2 = 140, L3 = 140, L4 = 54, L5 = 0.8; // Thông số vật lý của robot được trích xuất từ Solidworks
String inputString;  //chuỗi chứa dữ liệu được nhập từ bàn phím

void setup() {
  Serial.begin(115200);      //thiết lập giao thức UART
  inputString.reserve(200);  //khai báo vùng đệm chứa dữ liệu UART là 200 bytes
  //Thiết lập các chân input, output và trạng thái ban đầu của các chân output
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(lmPin[i], INPUT_PULLUP);
    pinMode(dirPin[i], OUTPUT);
    pinMode(pulsePin[i], OUTPUT);

    digitalWrite(dirPin[i], 0);
    digitalWrite(pulsePin[i], 0);
  }
}

void loop() {
  //Kiểm tra khi nào trong vùng nhớ đệm UART có dữ liệu thì đọc, và đọc đến kí tự kết thúc ")"
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();  // Đọc byte nhập vào và lưu trong "inChar"
    inputString += inChar;              // Cộng dồn byte inChar vào chuỗi "inputString"
    if (inChar == ')')                  // Nếu như gặp kí từ kết thúc là "z"
    {
      Serial.print(inputString);
      Serial.print(" ");
      stringCompleted = true;  // Thì set flag UART = true và dừng việc cộng dồn "inChar" và chuỗi "inputString"
      break;
    }
    //bắt đầu xử lý chuỗi dữ liệu nhận được
    if (stringCompleted == true) {
      stringCompleted = false;
      splitDataKinematic(inputString);  // bóc tách  chuỗi dữ liệu bằng hàm
      if (inputString[0] == 'H') {      //nếu kí tự bắt đầu là H thì câu lệnh đó set home
        Serial.println("Set home");
        for (uint8_t i = 0; i < 3; i++) {
          Sethome(i, pulseHome[i], 2000);
          angleNow[i] = homeAngle[i];
          pulseNow[i] = (homeAngle[i] * ration_convert(i)) / 360.0;
        }
        
      } else {
        if (inputString[0] == 'F') {  //nếu kí tự bắt đầu là F thì câu lệnh đó chạy động học thuận
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
          if (inputString[0] == 'I') {  //nếu kí tự bắt đầu là I thì câu lệnh đó yêu cầu chạy động học nghịch
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
            Serial.print(" is a ");  //còn không thuộc các trường hợp bên trên thì câu lệnh không đúng
            Serial.println("Incorrect command!");
          }
        }
      }
      inputString = "";
    }  

  }
}

void splitDataKinematic(String inputString) {
  //Xác định vị trí các kí tự ( và ) trong chuỗi
  uint8_t charStart = inputString.indexOf('(');
  uint8_t charEnd = inputString.indexOf(')');
  //Tách 2 kí tự ( và ) ra và chuỗi data_raw chỉ còn lại các số và dấu cách
  String data_raw = inputString.substring(charStart + 1, charEnd);

  //Xác định vị trí của dấu cách trong chuỗi data_raw
  uint8_t charSpace[] = { 0, 0 };
  uint8_t q = 0;
  for (uint8_t p = 0; p < data_raw.length(); p++) {
    if (data_raw[p] == ' ') {
      charSpace[q] = p;
      q++;
    }
  }

  //Nếu kí tự bắt đầu của chuỗi inputString thì bóc tách dữ liệu lưu vào positionSP
  if (inputString[0] == 'I') {
    positionSP[0] = (data_raw.substring(0, charSpace[0])).toDouble();
    positionSP[1] = (data_raw.substring(charSpace[0] + 1, charSpace[1])).toDouble();
    positionSP[2] = (data_raw.substring(charSpace[1] + 1, data_raw.length())).toDouble();
  }

  //Nếu kí tự bắt đầu của chuỗi inputString thì bóc tách dữ liệu lưu vào angleSP
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

  // ===== B2: quy đổi tọa độ =====
  double r = sqrt(Px*Px + Py*Py);
  double rw = r - (L4 + L5);   // offset ngang
  double s  = Pz - L1;         // offset đứng

  // ===== B3: kiểm tra workspace =====
  double D = (rw*rw + s*s - L2*L2 - L3*L3) / (2*L2*L3);

  if (D < -1.0 || D > 1.0) {
    return false; // ngoài workspace
  }
  if (D > 1) D = 1;
  if (D < -1) D = -1;
  // ===== B4: q3 (elbow-down) =====
  double s3 = -sqrt(1 - D*D);   // luôn âm → elbow-down
  double c3 = D;
  double theta3 = atan2(s3, c3);

  // ===== B5: q2 =====
  double theta2 = atan2(s, rw) - atan2(L3*s3, L2 + L3*c3);

  // ===== B6: đổi sang độ =====
  double q1 = theta1 * 180.0 / M_PI;
  double q2 = theta2 * 180.0 / M_PI;
  double q3 = theta3 * 180.0 / M_PI;

  // ===== B7: offset theo cơ cấu robot =====
  angleSP[0] = q1;
  angleSP[1] = q2;   // tùy cơ cấu bạn
  angleSP[2] = q3;

  return true;
}
void Sethome(uint8_t NoJoint,int32_t pulse, uint16_t timeDelay) {
  //Set hướng xoay là 0 và xoay động cơ liên tục cho đến khi công tắc hành trình thay đổi trạng thái
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
      delayMicroseconds(400);  //chờ 200ms sau đó kiểm tra lại trạng thái của limit switch = chống dội
      if (digitalRead(lmPin[NoJoint]) == 0) {
        delay(1000);
        flagHome = true;
        break;
      }
    }
  }

  //set hướng ngược lại và phát xung theo giá trị xung được setup bên trên
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

