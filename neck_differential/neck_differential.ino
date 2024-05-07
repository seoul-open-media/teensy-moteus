//——————————————————————————————————————————————————————————————————————————————

// Demonstrates how to use SetPositionWaitComplete to wait until the
// exact time that a trajectory motion is completed.  Intended to
// execute on a CANBed FD from longan labs.
//  * https://mjbots.com/products/moteus-r4-11
//  * https://www.longan-labs.cc/1030009.html
// ——————————————————————————————————————————————————————————————————————————————

#include <ACAN2517FD.h>
#include <Moteus.h>
#define HWSERIAL Serial2
#define BUFFER_SIZE 1024

#define SSID  "seoulopenmedia"      // change this to match your WiFi SSID
#define PASS  "americano"  // change this to match your WiFi password

char buffer[BUFFER_SIZE];

// By default we are looking for OK\r\n
char OKrn[] = "OK\r\n";

//——————————————————————————————————————————————————————————————————————————————
//  The following pins are selected for the CANBed FD board.
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2517_SCK = 13 ; // SCK input of MCP2517
static const byte MCP2517_SDI =  11 ; // SDI input of MCP2517
static const byte MCP2517_SDO =  12 ; // SDO output of MCP2517

static const byte MCP2517_CS  = 10 ; // CS input of MCP2517
static const byte MCP2517_INT = 3 ; // INT output of MCP2517

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517FD can (MCP2517_CS, SPI, MCP2517_INT) ;

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 12;
  return options;
}());
Moteus moteus2(can, []() {
  Moteus::Options options;
  options.id = 7;
  return options;
}());

Moteus::PositionMode::Command position_cmd1;
Moteus::PositionMode::Command position_cmd2;
Moteus::PositionMode::Format position_fmt;

double pos1 = 0;
double pos2 = 0;

void setup() {
  pinMode (LED_BUILTIN, OUTPUT);

  // Let the world know we have begun!
  Serial.begin(115200);
  HWSERIAL.begin(9600);
  while (!Serial) {}
  Serial.println("Serial Ready");
  while (!HWSERIAL) {}
  Serial.println("HWSerial Ready");
  setupWiFi();
  // This operates the CAN-FD bus at 1Mbit for both the arbitration
  // and data rate.  Most arduino shields cannot operate at 5Mbps
  // correctly, so the moteus Arduino library permanently disables
  // BRS.

  SPI.begin();
  ACAN2517FDSettings settings(
      ACAN2517FDSettings::OSC_40MHz, 1000ll * 1000ll, DataBitRateFactor::x1);

//   // The atmega32u4 on the CANbed has only a tiny amount of memory.
//   // The ACAN2517FD driver needs custom settings so as to not exhaust
//   // all of SRAM just with its buffers.
  settings.mArbitrationSJW = 2;
  settings.mDriverTransmitFIFOSize = 1;
  settings.mDriverReceiveFIFOSize = 2;
  const uint32_t errorCode = can.begin(settings, [] { can.isr(); });
  
  // while (errorCode != 0) {
  //   Serial.print(F("CAN error 0x"));
  //   Serial.println(errorCode, HEX);
  //   delay(1000);
  // }
//  First we'll clear faults.
  moteus1.SetStop();
  moteus2.SetStop();
  Serial.println(F("all stopped"));


  position_fmt.position = Moteus::kInt16;
  position_fmt.velocity = Moteus::kInt16;
  position_fmt.maximum_torque = Moteus::kInt16;
  position_fmt.feedforward_torque = Moteus::kIgnore;
  position_fmt.kp_scale = Moteus::kIgnore;
  position_fmt.kd_scale = Moteus::kIgnore;
  position_fmt.stop_position = Moteus::kInt16;
  position_fmt.velocity_limit = Moteus::kInt16;
  position_fmt.accel_limit = Moteus::kInt16;

  position_cmd1.velocity_limit = 0.5;
  position_cmd1.accel_limit = 0.1;

  position_cmd2.velocity_limit = 0.5;
  position_cmd2.accel_limit = 0.1;

auto print_state = [&]() {
    const auto& v = moteus1.last_result().values;
    Serial.print(F(" mode="));
    Serial.print(static_cast<int>(v.mode));
    Serial.print(F(" pos="));
    Serial.print(v.position, 3);
    Serial.print(F(" vel="));
    Serial.print(v.velocity, 3);
    Serial.print(F(" torque="));
    Serial.print(v.torque, 3);
    Serial.print(F(" q_current="));
    Serial.print(v.q_current, 3);
    Serial.print(F(" d_current="));
    Serial.print(v.d_current, 3);
    Serial.print(F(" abs_position="));
    Serial.print(v.abs_position, 3);
    Serial.print(F(" motor_temperature="));
    Serial.print(v.motor_temperature, 3);
    Serial.print(F(" voltage="));
    Serial.print(v.voltage, 3);
    Serial.println();

    const auto& p = moteus2.last_result().values;
    Serial.print(F(" mode="));
    Serial.print(static_cast<int>(p.mode));
    Serial.print(F(" pos="));
    Serial.print(p.position, 3);
    Serial.print(F(" vel="));
    Serial.print(p.velocity, 3);
    Serial.print(F(" torque="));
    Serial.print(p.torque, 3);
    Serial.print(F(" q_current="));
    Serial.print(p.q_current, 3);
    Serial.print(F(" d_current="));
    Serial.print(p.d_current, 3);
    Serial.print(F(" abs_position="));
    Serial.print(p.abs_position, 3);
    Serial.print(F(" motor_temperature="));
    Serial.print(p.motor_temperature, 3);
    Serial.print(F(" voltage="));
    Serial.print(p.voltage, 3);
    Serial.println();
  };
  print_state();
}

void loop() {
  
  auto print_state = [&]() {
    const auto& v = moteus1.last_result().values;
    Serial.print(F(" mode="));
    Serial.print(static_cast<int>(v.mode));
    Serial.print(F(" pos="));
    Serial.print(v.position, 3);
    Serial.print(F(" vel="));
    Serial.print(v.velocity, 3);
    Serial.print(F(" torque="));
    Serial.print(v.torque, 3);
    Serial.print(F(" q_current="));
    Serial.print(v.q_current, 3);
    Serial.print(F(" d_current="));
    Serial.print(v.d_current, 3);
    Serial.print(F(" abs_position="));
    Serial.print(v.abs_position, 3);
    Serial.print(F(" motor_temperature="));
    Serial.print(v.motor_temperature, 3);
    Serial.print(F(" voltage="));
    Serial.print(v.voltage, 3);
    Serial.println();

    const auto& p = moteus2.last_result().values;
    Serial.print(F(" mode="));
    Serial.print(static_cast<int>(p.mode));
    Serial.print(F(" pos="));
    Serial.print(p.position, 3);
    Serial.print(F(" vel="));
    Serial.print(p.velocity, 3);
    Serial.print(F(" torque="));
    Serial.print(p.torque, 3);
    Serial.print(F(" q_current="));
    Serial.print(p.q_current, 3);
    Serial.print(F(" d_current="));
    Serial.print(p.d_current, 3);
    Serial.print(F(" abs_position="));
    Serial.print(p.abs_position, 3);
    Serial.print(F(" motor_temperature="));
    Serial.print(p.motor_temperature, 3);
    Serial.print(F(" voltage="));
    Serial.print(p.voltage, 3);
    Serial.println();
  };

  const auto& v = moteus1.last_result().values;
  const auto& p = moteus1.last_result().values;
  if(!v.position && !p.position){
    sendData(2, static_cast<int>(v.mode), v.position, v.abs_position, v.velocity, v.torque, v.q_current, v.d_current, v.voltage, v.temperature);
    sendData(3, static_cast<int>(p.mode),p.position, p.abs_position, p.velocity, p.torque, p.q_current, p.d_current, p.voltage, p.temperature);

  }
  // print_state();
  if(Serial.available()){
    sendData(1,0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,Serial.readStringUntil('\n').toFloat());
    // HWSERIAL.write(Serial.read());
    // String val = Serial.readStringUntil('\n');
    // if (val.indexOf("stat") != -1){
    //   Serial.println(can.available());
    //   print_state();
    //   return;
    // }
    // int firstIndex = val.indexOf(' ');
    // if(firstIndex == -1){
    //   return;
    // }
    // String firstValue = val.substring(0,firstIndex);
    // String secondValue = val.substring(firstIndex+1);
    // double pos1 = firstValue.toFloat();
    // double pos2 = secondValue.toFloat();
    // Serial.println(pos1);
    // Serial.println(pos2);
    // run2(pos1, pos2);
    // print_state();
  }

  if(HWSERIAL.available()>=29){
    Serial.println("input");
    // for(int i=0; i<12; i++){
    //   HWSERIAL.read();
    // }
    // uint8_t byteDatas[12];
    // HWSERIAL.readBytes(byteDatas,12);
    // Serial.write(byteDatas,12);
    HWSERIAL.readStringUntil(':');
    uint8_t byteData;
    float floatsp[4];

    // 첫 번째 바이트를 읽습니다.
    byteData = HWSERIAL.read();

    // 이후 4개의 float 데이터를 읽습니다.
    for (int i = 0; i < 4; i++) {
        uint8_t bytes[4];
        for (int j = 0; j < 4; j++) {
            bytes[3-j] = HWSERIAL.read();  // 각 float 데이터의 바이트를 읽어 배열에 저장
        }
        // 바이트 배열을 float로 변환
        memcpy(&floatsp[i], bytes, sizeof(float));
    }

    HWSERIAL.flush();
  
    Serial.print(byteData);
    Serial.print(" ");
    Serial.print(floatsp[0],4);
    Serial.print(" ");
    Serial.print(floatsp[1],4);
    Serial.print(" ");
    Serial.print(floatsp[2],4);
    Serial.print(" ");
    Serial.println(floatsp[3],4);

    if(byteData == 2){
      pos1 = floatsp[0];
    }else if(byteData == 3){
      pos2 = floatsp[0];
    }
    // position_cmd1.velocity = floatsp[1];
    // position_cmd1.maximum_torque = floatsp[2];
    // position_cmd1.stop_position =  floatsp[3];
    // run3(pos1, pos2);
      
  }
  delay(10);
}

double GetSPIposition(Moteus a){
  return a.last_result().values.position;
}

double GetI2Cposition(Moteus a){
  double i2c = a.last_result().values.abs_position;
  if (i2c < -0.5)
      i2c += 1.0;
  if (i2c > 0.5)
      i2c -= 1.0;
  return i2c;
}

double deg2rad(double d){
  return d/180.0*M_PI;
}

void run2(double deg1, double deg2){
  double tilt = constrain(deg1, -90, 90);
  double pan = constrain(deg2, -90, 90);

  auto offset = (deg2rad(tilt)/(2.0*M_PI) - GetI2Cposition(moteus1)) - (deg2rad(pan)/(2.0*M_PI) - GetI2Cposition(moteus2));
  
  if (offset > 1.0)
    {
        offset -= 1.0;
    }
    else if (offset < -1.0)
    {
        offset += 1.0;
    }
  

  double pos1 = GetSPIposition(moteus1)+offset;

    
  auto offset2 = (deg2rad(tilt)/(2.0*M_PI) - GetI2Cposition(moteus1)) + (deg2rad(pan)/(2.0*M_PI) - GetI2Cposition(moteus2));
  
   if (offset2 > 1.0)
    {
        offset2 -= 1.0;
    }
    else if (offset2 < -1.0)
    {
        offset2 += 1.0;
    }

  double pos2 = GetSPIposition(moteus2)+offset2;


  setDoublePosition(pos1, pos2, 0.02);
  // result1 = moteus1.SetPosition(position_cmd1, &position_fmt);
  // result2 = moteus2.SetPosition(position_cmd2, &position_fmt);
  
}

void run3(double pos1, double pos2){
  
  auto offset = (pos1/(2.0*M_PI) - GetI2Cposition(moteus1)) - (pos2/(2.0*M_PI) - GetI2Cposition(moteus2));
  
  if (offset > 1.0)
    {
        offset -= 1.0;
    }
    else if (offset < -1.0)
    {
        offset += 1.0;
    }
  

  double pos_1 = GetSPIposition(moteus1)+offset;

    
  auto offset2 = (pos1/(2.0*M_PI) - GetI2Cposition(moteus1)) + (pos2/(2.0*M_PI) - GetI2Cposition(moteus2));
  
   if (offset2 > 1.0)
    {
        offset2 -= 1.0;
    }
    else if (offset2 < -1.0)
    {
        offset2 += 1.0;
    }

  double pos_2 = GetSPIposition(moteus2)+offset2;


  setDoublePosition(pos_1, pos_2, 0.02);
  // result1 = moteus1.SetPosition(position_cmd1, &position_fmt);
  // result2 = moteus2.SetPosition(position_cmd2, &position_fmt);
  
}


void setDoublePosition(double pos1, double pos2, double period_s){
  position_cmd1.position = pos1;
  position_cmd2.position = pos2;

  int count = 2;
  while (true) {
      const bool got_result1 = moteus1.SetPosition(position_cmd1, &position_fmt);
      const bool got_result2 = moteus2.SetPosition(position_cmd2, &position_fmt);
      if (got_result1 && got_result2) {
        count = count - 1;
        if (count < 0) { count = 0; }
      }
      if (count == 0 &&
          got_result1 && got_result2 &&
          moteus1.last_result().values.trajectory_complete && moteus2.last_result().values.trajectory_complete) {
        return;
      }

      delay(static_cast<unsigned long>(period_s * 1000));
    }
}

void run(double pos1, double pos2){
  moteus1.Poll();
  moteus2.Poll();

  // double m13_pos_cmd = constrain(pos1, -1.0, 1.0);
  // double m14_pos_cmd = constrain(pos2, -1.0, 1.0);

  double m13_pos_cmd = deg2rad(pos1);
  double m14_pos_cmd = deg2rad(pos2);

  auto offset = (m13_pos_cmd / (2.0 * M_PI) - GetI2Cposition(moteus1)) + (m14_pos_cmd / (2.0 * M_PI) - GetI2Cposition(moteus2));
    if (offset > 1.0)
    {
        offset -= 1.0;
    }
    else if (offset < -1.0)
    {
        offset += 1.0;
    }
  position_cmd1.position = GetSPIposition(moteus1) + offset;

  Serial.println(position_cmd1.position);

  auto offset2 = (m13_pos_cmd / (2.0 * M_PI) - GetI2Cposition(moteus1)) - (m14_pos_cmd / (2.0 * M_PI) - GetI2Cposition(moteus2));

    if (offset2 > 1.0)
    {
        offset2 -= 1.0;
    }
    else if (offset2 < -1.0)
    {
        offset2 += 1.0;
    }

    position_cmd2.position = GetSPIposition(moteus2) + offset2;

    Serial.println(position_cmd2.position);
    moteus1.SetPosition(position_cmd1, &position_fmt);
    moteus2.SetPosition(position_cmd2, &position_fmt);
}

byte wait_for_esp_response(int timeout, char* term=OKrn) {
  unsigned long t=millis();
  bool found=false;
  int i=0;
  int len=strlen(term);
  // wait for at most timeout milliseconds
  // or if OK\r\n is found
  while(millis()<t+timeout) {
    if(HWSERIAL.available()) {
      buffer[i++]=HWSERIAL.read();
      if(i>=len) {
        if(strncmp(buffer+i-len, term, len)==0 || strchr(buffer+i-len, '>') != NULL) {
          found=true;
          break;
        }
      }
    }
  }
  buffer[i]=0;
  Serial.print(buffer);
  return found;
}


bool read_till_eol() {
  static int i=0;
  while(HWSERIAL.available()) {
    buffer[i++]=HWSERIAL.read();
    if(i==BUFFER_SIZE)  i=0;
    if(i>1 && buffer[i-2]==13 && buffer[i-1]==10) {
      buffer[i]=0;
      i=0;
      Serial.print(buffer);
      return true;
    }
  }
  return false;
}

void setupWiFi() {

  HWSERIAL.println("ATE0");
  wait_for_esp_response(5000);
  HWSERIAL.println("AT+CIPMUX=1");
  wait_for_esp_response(5000); 
  HWSERIAL.println("AT+CIPCLOSE=4");
  wait_for_esp_response(5000);
  // turn on echo
  // HWSERIAL.println("ATE1");
  // wait_for_esp_response(5000);
  // HWSERIAL.println("AT");
  // wait_for_esp_response(5000);

  // // set mode 1 (client)
  // HWSERIAL.println("AT+CWJAP?");
  // wait_for_esp_response(5000); 
  


  HWSERIAL.println("AT+CIPSTART=4,\"UDP\",\"192.168.0.22\",9999,9998,0");
  wait_for_esp_response(5000); 

  // HWSERIAL.println("AT+CIPSEND=4,7");
  // wait_for_esp_response(5000); 
  // HWSERIAL.write("UDPtest");
  // wait_for_esp_response(5000);

  // Serial.println("ready");
}

void sendData(int motorId, int state, float pos, float i2c_pos, float velocity, float torque, float q_current, float d_current, float voltage, float temperature){

  uint8_t bytes[4] = {motorId, 0, 0, state}; // 바이트 데이터
  float floats[8] = {pos, i2c_pos, velocity, torque, q_current, d_current, voltage, temperature}; // 실수 데이터

  unsigned char buffer_data[4 + 8 * sizeof(float)];  // 바이트 배열 준비
  int length = 4 + 8 * sizeof(float);
  // 바이트 데이터 복사
  memcpy(buffer_data, bytes, 4);

  // 실수 데이터 복사, 빅 엔디안 변환 필요
  for (int i = 0; i < 8; i++) {
      uint32_t* floatAsInt = (uint32_t*)&floats[i]; // float를 uint32_t로 취급
      uint32_t bigEndianValue = htonl(floatAsInt); // 호스트 엔디안을 네트워크 엔디안(빅 엔디안)으로 변환
      memcpy(buffer_data + 4 + i * sizeof(float), &bigEndianValue, sizeof(float));
  }
  HWSERIAL.println("AT+CIPSEND=4,36");
  // read_till_eol();
  // read_till_eol();
  wait_for_esp_response(100);
  wait_for_esp_response(100);
  // 버퍼 출력
  // for (int i = 0; i < sizeof(buffer_data); i++) {
  //     HWSERIAL.print(buffer_data[i], HEX);
  // }
  // HWSERIAL.println("");
  HWSERIAL.write(buffer_data, sizeof(buffer_data));
  for (int i = 0; i < sizeof(buffer_data); i++) {
      Serial.print(buffer_data[i], HEX);
  }
  Serial.println("");
  wait_for_esp_response(5000);
  return 0;
}

uint32_t htonl(uint32_t* hostlong) {
    uint8_t *s = (uint8_t*)hostlong;
    return (uint32_t)(s[0] << 24) | (s[1] << 16) | (s[2] << 8) | s[3];
}