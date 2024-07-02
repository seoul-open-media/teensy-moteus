//——————————————————————————————————————————————————————————————————————————————

// Demonstrates how to use SetPositionWaitComplete to wait until the
// exact time that a trajectory motion is completed.  Intended to
// execute on a CANBed FD from longan labs.
//  * https://mjbots.com/products/moteus-r4-11
//  * https://www.longan-labs.cc/1030009.html
// ——————————————————————————————————————————————————————————————————————————————

#include <ACAN2517FD.h>
#include <Moteus.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define HWSERIAL Serial2
#define BUFFER_SIZE 1024

#define SSID "seoulopenmedia"  // change this to match your WiFi SSID
#define PASS "americano"       // change this to match your WiFi password

char buffer[BUFFER_SIZE];
const int PACKET_SIZE = 34;
byte HWbuffer[PACKET_SIZE];
int bufferIndex = 0;

// By default we are looking for OK\r\n
char OKrn[] = "OK\r\n";

const int pwmPin1 = 2;
const int a1Pin1 = 4;
const int b1Pin1 = 5;
const int pwmPin2 = 6;
const int a1Pin2 = 40;
const int b1Pin2 = 41;

int left_leg = 0;
int right_leg = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire1);
Adafruit_BNO055 bno1 = Adafruit_BNO055(-1, 0x28, &Wire);

//——————————————————————————————————————————————————————————————————————————————
//  The following pins are selected for the CANBed FD board.
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2517_SCK = 13;  // SCK input of MCP2517
static const byte MCP2517_SDI = 11;  // SDI input of MCP2517
static const byte MCP2517_SDO = 12;  // SDO output of MCP2517

static const byte MCP2517_CS = 10;  // CS input of MCP2517
static const byte MCP2517_INT = 3;  // INT output of MCP2517

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517FD Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 11;
  return options;
}());
Moteus moteus2(can, []() {
  Moteus::Options options;
  options.id = 12;
  return options;
}());

Moteus::PositionMode::Command position_cmd1;
Moteus::PositionMode::Command position_cmd2;
Moteus::PositionMode::Format position_fmt;

double pos1 = 0;
double pos2 = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(a1Pin1, OUTPUT);
  pinMode(b1Pin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(a1Pin2, OUTPUT);
  pinMode(b1Pin2, OUTPUT);
  digitalWrite(pwmPin1, 0);
  digitalWrite(a1Pin1, 0);
  digitalWrite(b1Pin1, 0);
  digitalWrite(pwmPin2, 0);
  digitalWrite(a1Pin2, 0);
  digitalWrite(b1Pin2, 0);

  // Let the world know we have begun!
  Serial.begin(115200);
  HWSERIAL.begin(115200);
  while (!Serial) {}
  Serial.println("Serial Ready");
  while (!HWSERIAL) {}
  Serial.println("HWSerial Ready");
  // setupWiFi();
  // This operates the CAN-FD bus at 1Mbit for both the arbitration
  // and data rate.  Most arduino shields cannot operate at 5Mbps
  // correctly, so the moteus Arduino library permanently disables
  // BRS.

  // Serial.println("Orientation Sensor Raw Data Test");
  // Serial.println("");

  // /* Initialise the sensor */
  // if (!bno.begin() || !bno1.begin()) {
  //   /* There was a problem detecting the BNO055 ... check your connections */
  //   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //   while (1)
  //     ;
  // }

  // delay(1000);

  // bno.setExtCrystalUse(true);

  // Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");


  SPI.begin();
  ACAN2517FDSettings settings(
    ACAN2517FDSettings::OSC_40MHz, 1000ll * 1000ll, DataBitRateFactor::x1);

  //   // The atmega32u4 on the CANbed has only a tiny amount of memory.
  //   // The ACAN2517FD driver needs custom settings so as to not exhaust
  //   // all of SRAM just with its buffers.
  settings.mArbitrationSJW = 2;
  settings.mDriverTransmitFIFOSize = 1;
  settings.mDriverReceiveFIFOSize = 2;
  const uint32_t errorCode = can.begin(settings, [] {
    can.isr();
  });

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
  position_fmt.velocity = Moteus::kIgnore;
  position_fmt.maximum_torque = Moteus::kInt16;
  position_fmt.feedforward_torque = Moteus::kIgnore;
  position_fmt.kp_scale = Moteus::kIgnore;
  position_fmt.kd_scale = Moteus::kIgnore;
  position_fmt.stop_position = Moteus::kIgnore;
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

  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  if (gyro == 3) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);

    // /* Display the floating point data */
    // Serial.print("X: ");
    // Serial.print(euler.x());
    // Serial.print(" Y: ");
    // Serial.print(euler.y());
    // Serial.print(" Z: ");
    // Serial.print(euler.z());
    // Serial.print("\t\t");
    // Serial.print("X: ");
    // Serial.print(euler1.x());
    // Serial.print(" Y: ");
    // Serial.print(euler1.y());
    // Serial.print(" Z: ");
    // Serial.print(euler1.z());
    // Serial.println("");
  }

  // print_state();
  if (Serial.available()) {
    // float a = Serial.readStringUntil('\n').toFloat();
    // sendData(13,0,1.1,2.2,3.3,4.4,5.5,6.6,7.7,a);
    // sendData(14,0,1.1,2.2,3.3,4.4,5.5,6.6,7.7,a+1);

    // HWSERIAL.println(Serial.read());

    String val = Serial.readStringUntil('\n');
    if (val.indexOf("stat") != -1) {
      Serial.println(can.available());
      print_state();
      return;
    }
    int firstIndex = val.indexOf(' ');
    if (firstIndex == -1) {
      if (val.toInt() == 1) {
        if (left_leg == 0) {
          digitalWrite(a1Pin1, 1);
          digitalWrite(pwmPin1, 1);
          left_leg = 1;
        } else {

          digitalWrite(a1Pin1, 0);

          digitalWrite(pwmPin1, 0);
          left_leg = 0;
        }
        return;
      }
      if (val.toInt() == 2) {
        if (right_leg == 0) {
          digitalWrite(a1Pin2, 1);
          digitalWrite(pwmPin2, 1);
          right_leg = 1;
        } else {
          digitalWrite(a1Pin2, 0);
          digitalWrite(pwmPin2, 0);
          right_leg = 0;
        }
        return;
      }
      if (val.toInt() == 3) {
        //sequence
        double origin1 = moteus1.last_result().values.position;
        double origin2 = moteus2.last_result().values.position;

        run2walk(1, 0);

        run2walk(2, 0);
        run2walk(1, 0.12);
        print_state();
        for (int i = 0; i < 2; i++) {

          run2walk(2, 0.3);
          run2walk(2, 0.12);
          print_state();
          run2walk(1, 0.3);
          run2walk(1, 0.12);
          print_state();
          // stay2walk(100);
          // print_state();
        }
        // run2walk(2, -0.2);
        // print_state();
        moteus1.SetStop();
        moteus2.SetStop();
        return;
      }
      if (val.toInt() == 4) {
        //sequence
        double origin1 = moteus1.last_result().values.position;
        double origin2 = moteus2.last_result().values.position;

        run2walk(1, 0);

        run2walk(2, 0);
        run2walk(1, -0.15);
        print_state();
        for (int i = 0; i < 4; i++) {

          run2walk(2, -0.3);
          run2walk(2, -0.12);
          print_state();
          run2walk(1, -0.3);
          run2walk(1, -0.12);
          print_state();
          // stay2walk(100);
          // print_state();
        }
        // run2walk(2, -0.2);
        // print_state();
        moteus1.SetStop();
        moteus2.SetStop();
        return;
      }
      if (val.toInt() == 5) {
        moteus1.SetStop();
        moteus2.SetStop();
        return;
      }
      if (val.toInt() == 6) {
        stay2walk(500);
        return;
      }
      if (val.toInt() == 7) {
        stay2walk(3000);
        return;
      }
      if (val.toInt() == 8) {
        run2walk(1, 0.5);
        double error = 5;
        while (1) {
          bno.getCalibration(&system, &gyro, &accel, &mag);
          if (gyro == 3) {
            imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
            while ((euler.x() == 0 && euler.y() == 0 && euler.z() == 0) || (euler1.x() == 0 && euler1.y() == 0 && euler1.z() == 0)) {
              euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
              euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
            }
            double angle = euler.y() - 150;
            target2walk1(angle, 0.01,0.5,0);
          }
        }
        free2walk2();
        return;
      }
      if (val.toInt() == 9) {
        delay(1000);
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
        while ((euler.x() == 0 && euler.y() == 0 && euler.z() == 0) || (euler1.x() == 0 && euler1.y() == 0 && euler1.z() == 0)) {
          euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
          euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
        }

        double speed = 0.01;
        double error = 0.3;
        double zero_1 = euler.y();
        double zero_2 = euler1.y();
        double angle_origin = 3.5;
        double angle = angle_origin;
        double angle1 = zero_1 - angle_origin;
        double angle2 = zero_2 - angle_origin;
        angle1 = zero_1 - angle_origin;
        Serial.println("left");
        Serial.println(angle1);
        // run2walk2(1,0.2);
        target2walk1(angle1, speed, error,0);
        stay2walk(50);
        delay(100);

        for (int i = 0; i < 5; i++) {

          angle2 = zero_2 - angle_origin;

          Serial.println("right");
          Serial.println(angle2);
          target2walk2(angle2, speed, error,0);
          stay2walk(50);
          delay(100);

          // angle1 = zero_1 - angle_origin;
          angle1 = zero_1 - angle_origin;
          Serial.println("left");
          Serial.println(angle1);
          target2walk1(angle1, speed, error,0);
          stay2walk(50);
          delay(100);
        }



        angle2 = zero_2;
        Serial.println("right");
        Serial.println(angle2);
        target2walk2(angle2, speed, error, 0);
        stay2walk(50);
        delay(100);
        return;
      }
      if (val.toInt() == 10) {
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
        double target = 150.0;
        Serial.print(" X: ");
        Serial.print(euler.x());
        Serial.print(" Y: ");
        Serial.print(euler.y());
        Serial.print(" Z: ");
        Serial.print(euler.z());
        Serial.print("\t\t");

        Serial.print(" X: ");
        Serial.print(euler1.x());
        Serial.print(" Y: ");
        Serial.print(euler1.y());
        Serial.print(" Z: ");
        Serial.print(euler1.z());
        Serial.println("");
      }
      if (val.toInt() == 11) {
        for(int i=0;i<5;i++){
          stay2walk(50);
          free2walk2();
          delay(100);
        }
      }
      if (val.toInt() == 12) {
        for(int i=0 ; i<5;i++){
          run2walk(1, 0.01);
          delay(5);
          run2walk(1, -0.01);
          delay(5);
        }
        
      }
      
      return;
    }


    String firstValue = val.substring(0, firstIndex);
    String secondValue = val.substring(firstIndex + 1);

    if (firstValue.toInt() == 1 || firstValue.toInt() == 2) {
      bno.getCalibration(&system, &gyro, &accel, &mag);

      if (gyro == 3) {
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
        /*Real Position*/
        run2walk(firstValue.toInt(), secondValue.toFloat());

        Serial.print(" X: ");
        Serial.print(euler.x());
        Serial.print(" Y: ");
        Serial.print(euler.y());
        Serial.print(" Z: ");
        Serial.print(euler.z());
        Serial.print("\t\t");

        Serial.print(" X: ");
        Serial.print(euler1.x());
        Serial.print(" Y: ");
        Serial.print(euler1.y());
        Serial.print(" Z: ");
        Serial.print(euler1.z());
        Serial.println("");
        print_state();
      }
      return;
    }
    if (firstValue.toInt() == 3 || firstValue.toInt() == 4) {
      diff2walk(firstValue.toInt() - 2, secondValue.toFloat());
      return;
    }
    if (firstValue.toInt() == 5) {
      run2walk2(1, 0.1);
      target2walk1(secondValue.toFloat(), 0.015, 0.5, 0);
      free2walk2();
      return;
    }
    if (firstValue.toInt() == 6) {
      run2walk2(2, 0.1);
      target2walk2(secondValue.toFloat(), 0.015, 0.5, 0);
      free2walk2();
      return;
    }

    /*neck differential*/
    // double pos1 = firstValue.toFloat();
    // double pos2 = secondValue.toFloat();
    // Serial.println(pos1);
    // Serial.println(pos2);
    // run2(pos1, pos2);
    // print_state();
    // const auto& v = moteus1.last_result().values;
    // const auto& p = moteus2.last_result().values;
    // sendData(13, static_cast<int>(v.mode), v.position, v.abs_position, v.velocity, v.torque, v.q_current, v.d_current, v.voltage, v.motor_temperature);
    // sendData(14, static_cast<int>(p.mode),p.position, p.abs_position, p.velocity, p.torque, p.q_current, p.d_current, p.voltage, p.motor_temperature);
  }
  if (HWSERIAL.available()) {
    // Serial.println(HWSERIAL.read());
    uint8_t byteData = HWSERIAL.read();
    // Serial.print(byteData);
    // Serial.print(" ");
    buffer[bufferIndex++] = byteData;

    if (bufferIndex == PACKET_SIZE) {
      processPacket(buffer);
      bufferIndex = 0;
      // Serial.println();
    }
  }
  delay(10);
}

void stay2walk(int second) {
  Serial.println("stay");
  digitalWrite(a1Pin1, 1);
  digitalWrite(pwmPin1, 1);
  digitalWrite(a1Pin2, 1);
  digitalWrite(pwmPin2, 1);
  delay(second);
  digitalWrite(a1Pin1, 0);
  digitalWrite(pwmPin1, 0);
  digitalWrite(a1Pin2, 0);
  digitalWrite(pwmPin2, 0);
}

void run2walk(int motor, double pos) {

  position_cmd1.velocity_limit = 0.05;
  position_cmd1.accel_limit = 0.05;
  position_cmd1.maximum_torque = 10.0;
  position_cmd1.velocity = 0.05;
  position_cmd2.velocity_limit = 0.05;
  position_cmd2.accel_limit = 0.05;
  position_cmd2.maximum_torque = 10.0;
  position_cmd2.velocity = 0.05;
  if (motor == 1) {
    Serial.println(pos);
    double origin1 = moteus1.last_result().values.position;
    double origin2 = moteus2.last_result().values.position;
    position_cmd1.position = origin1 - pos;
    // Serial.println(origin1 - pos);
    digitalWrite(a1Pin1, 1);
    digitalWrite(pwmPin1, 1);
    digitalWrite(a1Pin2, 0);
    digitalWrite(pwmPin2, 0);
    // moteus2.SetBrake();
    // moteus1.SetPositionWaitComplete(position_cmd1, 0.02, &position_fmt);
    delay(10);
    setDoublePosition(origin1 - pos, origin2, 0.001);
    // setDoublePosition(origin1 - pos, origin2, 0.01);
    // moteus2.SetBrake();
    // moteus1.SetPositionWaitComplete(position_cmd1, 0.02, &position_fmt);
    // delay(3000);
    digitalWrite(a1Pin1, 0);
    digitalWrite(pwmPin1, 0);
    digitalWrite(a1Pin2, 0);
    digitalWrite(pwmPin2, 0);
  } else if (motor == 2) {
    Serial.println(pos);
    double origin1 = moteus1.last_result().values.position;
    double origin2 = moteus2.last_result().values.position;
    position_cmd2.position = origin2 - pos;
    // Serial.println(origin2 - pos);
    digitalWrite(a1Pin1, 0);
    digitalWrite(pwmPin1, 0);
    digitalWrite(a1Pin2, 1);
    digitalWrite(pwmPin2, 1);
    delay(10);
    setDoublePosition(origin1, origin2 - pos, 0.001);
    // setDoublePosition(origin1, origin2 - pos, 0.01);
    // moteus1.SetBrake();
    // moteus2.SetPositionWaitComplete(position_cmd2, 0.02, &position_fmt);

    // // delay(3000);
    digitalWrite(a1Pin1, 0);
    digitalWrite(pwmPin1, 0);
    digitalWrite(a1Pin2, 0);
    digitalWrite(pwmPin2, 0);
  }
}

void target2walk1(double target, double speed, double error, double zero) {
  while (1) {

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    if (gyro == 3) {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
      Serial.print(" X: ");
      Serial.print(euler.x());
      Serial.print(" Y: ");
      Serial.print(euler.y());
      Serial.print(" Z: ");
      Serial.print(euler.z());
      Serial.print("\t\t");

      Serial.print(" X: ");
      Serial.print(euler1.x());
      Serial.print(" Y: ");
      Serial.print(euler1.y());
      Serial.print(" Z: ");
      Serial.print(euler1.z());
      Serial.println("");
      if (euler.x() == 0 && euler.y() == 0 && euler.z() == 0) {
        continue;
      }
      if (target > 90) {
        if (((target - (180 - euler.y() * (-1))) < error && (target - (180 - euler.y() * (-1))) > -1 * error) && (euler.z() < 0.0)) {
          break;
        }
        if ((euler.z() > 0.0)) {
          run2walk2(1, speed);
        } else {
          if (target - (180 - euler.y() * (-1)) > error) {
            run2walk2(1, speed);
          } else if (target - (180 - euler.y() * (-1)) < -1 * error) {
            run2walk2(1, -1 * speed);
          } else {
            break;
          }
        }
      } else {
          double e = target - euler.y();
            if (e > error) {
              run2walk2(1, -1 * speed);
            } else if (e < -1 * error) {
              run2walk2(1, speed);
            } else {
              break;
            }
          
        
      }
      // if((target - euler1.y()) > 5.0 && && (euler1.z() < 0.0)){
      //   run2walk(1, 0.02);
      // }else if((target - euler1.y()) > 5.0 && && (euler1.z() < 0.0))
      // else if((target - euler1.y()) < -5.0){
      //   run2walk(1, -0.02);
      // }
    }
  }
  free2walk2();
  return;
}

void target2walk2(double target, double speed, double error, double zero) {
  while (1) {

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    if (gyro == 3) {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu::Vector<3> euler1 = bno1.getVector(Adafruit_BNO055::VECTOR_EULER);
      Serial.print(" X: ");
      Serial.print(euler.x());
      Serial.print(" Y: ");
      Serial.print(euler.y());
      Serial.print(" Z: ");
      Serial.print(euler.z());
      Serial.print("\t\t");

      Serial.print(" X: ");
      Serial.print(euler1.x());
      Serial.print(" Y: ");
      Serial.print(euler1.y());
      Serial.print(" Z: ");
      Serial.print(euler1.z());
      Serial.println("");
      if (euler1.x() == 0 && euler1.y() == 0 && euler1.z() == 0) {
        continue;
      }
      if (target > 90) {
        if (((target - (180 - euler1.y()*(-1))) < error && (target - (180 - euler1.y()*(-1))) > -1 * error) && (euler1.z() > 0.0)) {
          break;
        }
        if ((euler1.z() < 0.0)) {
          run2walk2(2, speed);
        } else {
          if (target - (180 - euler1.y()*(-1)) > error) {
            run2walk2(2, speed);
          } else if (target - (180 - euler1.y()*(-1)) < -1 * error) {
            run2walk2(2, -1 * speed);
          } else {
            break;
          }
        }
      } else {
          double e = target - euler1.y();
            if (e > error) {
              run2walk2(2, -1 * speed);
            } else if (e < -1 * error) {
              run2walk2(2, speed);
            } else {
              break;
            }
          
        
        
      }
      // if((target - euler1.y()) > 5.0 && && (euler1.z() < 0.0)){
      //   run2walk(1, 0.02);
      // }else if((target - euler1.y()) > 5.0 && && (euler1.z() < 0.0))
      // else if((target - euler1.y()) < -5.0){
      //   run2walk(1, -0.02);
      // }
    }
  }
  free2walk2();
  return;
}

void diff2walk(int motor, double pos) {

  position_cmd1.velocity_limit = 2.0;
  position_cmd1.accel_limit = 2.0;
  position_cmd1.maximum_torque = 4.0;
  position_cmd1.velocity = 2.0;
  position_cmd2.velocity_limit = 2.0;
  position_cmd2.accel_limit = 2.0;
  position_cmd2.maximum_torque = 4.0;
  position_cmd2.velocity = 2.0;
  if (motor == 1) {
    Serial.println(pos);
    double origin1 = moteus1.last_result().values.position;
    double origin2 = moteus2.last_result().values.position;
    position_cmd1.position = origin1 - pos;
    position_cmd2.position = origin2 + pos;
    // setDoublePosition(origin1 - pos,origin2 + pos,0.01);
    moteus1.SetPosition(position_cmd1, &position_fmt);
    moteus2.SetPosition(position_cmd2, &position_fmt);
    delay(100);
    // delay(3000);
  } else if (motor == 2) {
    Serial.println(pos);
    double origin1 = moteus1.last_result().values.position;
    double origin2 = moteus2.last_result().values.position;
    position_cmd1.position = origin1 + pos;
    position_cmd2.position = origin2 - pos;
    // setDoublePosition(origin1 + pos,origin2 - pos,0.01);
    moteus1.SetPosition(position_cmd1, &position_fmt);
    moteus2.SetPosition(position_cmd2, &position_fmt);
    delay(100);
  }
}

void run2walk2(int motor, double pos) {

  position_cmd1.velocity_limit = 5.0;
  position_cmd1.accel_limit = 3.0;
  position_cmd1.maximum_torque = 10.0;
  position_cmd1.velocity = 4.0;
  position_cmd2.velocity_limit = 5.0;
  position_cmd2.accel_limit = 3.0;
  position_cmd2.maximum_torque = 10.0;
  position_cmd2.velocity = 4.0;
  if (motor == 1) {
    Serial.println(pos);
    double origin1 = moteus1.last_result().values.position;
    double origin2 = moteus2.last_result().values.position;
    position_cmd1.position = origin1 - pos;
    position_cmd2.position = origin2 + pos;
    // Serial.println(origin1 - pos);
    digitalWrite(a1Pin1, 1);
    digitalWrite(pwmPin1, 1);
    digitalWrite(a1Pin2, 0);
    digitalWrite(pwmPin2, 0);
    // moteus2.SetBrake();
    // moteus1.SetPositionWaitComplete(position_cmd1, 0.02, &position_fmt);
    delay(10);
    moteus1.SetPosition(position_cmd1, &position_fmt);
    moteus2.SetPosition(position_cmd2, &position_fmt);
    delay(100);
    // setDoublePosition(origin1 - pos, origin2 + pos, 0.01);
    // setDoublePosition(origin1 - pos, origin2, 0.01);
    // moteus2.SetBrake();
    // moteus1.SetPositionWaitComplete(position_cmd1, 0.02, &position_fmt);
    // delay(3000);
    // digitalWrite(a1Pin1,0);
    // digitalWrite(pwmPin1, 0);
  } else if (motor == 2) {
    Serial.println(pos);
    double origin1 = moteus1.last_result().values.position;
    double origin2 = moteus2.last_result().values.position;
    position_cmd1.position = origin1 + pos;
    position_cmd2.position = origin2 - pos;
    // Serial.println(origin2 - pos);
    digitalWrite(a1Pin2, 1);
    digitalWrite(pwmPin2, 1);
    digitalWrite(a1Pin1, 0);
    digitalWrite(pwmPin1, 0);
    delay(10);
    moteus1.SetPosition(position_cmd1, &position_fmt);
    moteus2.SetPosition(position_cmd2, &position_fmt);
    delay(100);
    // setDoublePosition(origin1 + pos, origin2 - pos, 0.01);
    // setDoublePosition(origin1, origin2 - pos, 0.01);
    // moteus1.SetBrake();
    // moteus2.SetPositionWaitComplete(position_cmd2, 0.02, &position_fmt);

    // // delay(3000);
    // digitalWrite(a1Pin2,0);
    // digitalWrite(pwmPin2, 0);
  }
}



void free2walk2() {

  digitalWrite(a1Pin1, 0);
  digitalWrite(pwmPin1, 0);
  digitalWrite(a1Pin2, 0);
  digitalWrite(pwmPin2, 0);
}
void processPacket(byte* packet) {
  if (packet[0] == -1) {
    Serial.println("input error");
    return;
  }
  float floatsp1[4];
  for (int i = 0; i < 4; i++) {
    uint8_t bytes[4];
    for (int j = 0; j < 4; j++) {
      bytes[3 - j] = packet[4 * i + j + 1];  // 각 float 데이터의 바이트를 읽어 배열에 저장
                                             // Serial.print(bytes[3-j]);
                                             // Serial.print(" ");
    }
    // 바이트 배열을 float로 변환
    memcpy(&floatsp1[i], bytes, sizeof(float));
  }

  float floatsp2[4];
  for (int i = 0; i < 4; i++) {
    uint8_t bytes[4];
    for (int j = 0; j < 4; j++) {
      bytes[3 - j] = packet[4 * i + j + 18];  // 각 float 데이터의 바이트를 읽어 배열에 저장
                                              // Serial.print(bytes[3-j]);
                                              // Serial.print(" ");
    }
    // 바이트 배열을 float로 변환
    memcpy(&floatsp2[i], bytes, sizeof(float));
  }

  HWSERIAL.flush();

  if (!isnan(floatsp1[0]) && !isnan(floatsp1[1]) && !isnan(floatsp1[2]) && !isnan(floatsp1[3]) && !isnan(floatsp2[0]) && !isnan(floatsp2[1]) && !isnan(floatsp2[2]) && !isnan(floatsp2[3])) {
    if (packet[0] == 13) {
      position_cmd1.position = floatsp1[0];
      position_cmd1.velocity_limit = floatsp1[1];
      position_cmd1.maximum_torque = floatsp1[2];
      position_cmd1.accel_limit = floatsp1[3];

      position_cmd2.position = floatsp2[0];
      position_cmd2.velocity_limit = floatsp2[1];
      position_cmd2.maximum_torque = floatsp2[2];
      position_cmd2.accel_limit = floatsp2[3];
    } else if (packet[0] == 14) {
      position_cmd2.position = floatsp1[0];
      position_cmd2.velocity_limit = floatsp1[1];
      position_cmd2.maximum_torque = floatsp1[2];
      position_cmd2.accel_limit = floatsp1[3];

      position_cmd1.position = floatsp2[0];
      position_cmd1.velocity_limit = floatsp2[1];
      position_cmd1.maximum_torque = floatsp2[2];
      position_cmd1.accel_limit = floatsp2[3];
    }
    run3(position_cmd1.position, position_cmd2.position);
    const auto& v = moteus1.last_result().values;
    const auto& p = moteus2.last_result().values;
    sendData(13, static_cast<int>(v.mode), v.position, v.abs_position, v.velocity, v.torque, v.q_current, v.d_current, v.voltage, v.motor_temperature);
    sendData(14, static_cast<int>(p.mode), p.position, p.abs_position, p.velocity, p.torque, p.q_current, p.d_current, p.voltage, p.motor_temperature);
  }
  // Serial.print(packet[0]);
  // Serial.print(" ");
  // Serial.print(floatsp[0],4);
  // Serial.print(" ");
  // Serial.print(floatsp[1],4);
  // Serial.print(" ");
  // Serial.print(floatsp[2],4);
  // Serial.print(" ");
  // Serial.println(floatsp[3],4);
}

double GetSPIposition(Moteus a) {
  a.Poll();
  return a.last_result().values.position;
}

double GetI2Cposition(Moteus a) {
  a.Poll();
  double i2c = a.last_result().values.abs_position;
  if (i2c < -0.5)
    i2c += 1.0;
  if (i2c > 0.5)
    i2c -= 1.0;
  return i2c;
}

double deg2rad(double d) {
  return d / 180.0 * M_PI;
}

void run2(double deg1, double deg2) {

  position_cmd1.velocity_limit = 5.0;
  position_cmd1.accel_limit = 10.0;
  position_cmd1.maximum_torque = 50.0;

  position_cmd2.velocity_limit = 5.0;
  position_cmd2.accel_limit = 10.0;
  position_cmd2.maximum_torque = 50.0;

  double tilt = constrain(deg1, -90, 90);
  double pan = constrain(deg2, -180, 180);

  auto offset = (deg2rad(tilt) / (2.0 * M_PI) - GetI2Cposition(moteus1)) - (deg2rad(pan) / (2.0 * M_PI) - GetI2Cposition(moteus2));
  if (offset > 1.0) {
    offset -= 1.0;
  } else if (offset < -1.0) {
    offset += 1.0;
  }

  double pos1 = GetSPIposition(moteus1) + offset;

  auto offset2 = (deg2rad(tilt) / (2.0 * M_PI) - GetI2Cposition(moteus1)) + (deg2rad(pan) / (2.0 * M_PI) - GetI2Cposition(moteus2));
  if (offset2 > 1.0) {
    offset2 -= 1.0;
  } else if (offset2 < -1.0) {
    offset2 += 1.0;
  }

  double pos2 = GetSPIposition(moteus2) + offset2;
  setDoublePosition(pos1, pos2, 0.02);
  // result1 = moteus1.SetPosition(position_cmd1, &position_fmt);
  // result2 = moteus2.SetPosition(position_cmd2, &position_fmt);
}


void run3(double pos1, double pos2) {
  // Serial.println("moved");
  pos1 = constrain(pos1, -M_PI / 2, M_PI / 2);
  // pos2 = constrain(pos2, -M_PI*2, M_PI*2);
  // pos2 *=2.0 ;
  auto offset = (pos1 / (2.0 * M_PI) - GetI2Cposition(moteus1)) - (pos2 / (2.0 * M_PI) - GetI2Cposition(moteus2));

  if (offset > 1.0) {
    offset -= 1.0;
  } else if (offset < -1.0) {
    offset += 1.0;
  }


  double pos_1 = GetSPIposition(moteus1) + offset;


  auto offset2 = (pos1 / (2.0 * M_PI) - GetI2Cposition(moteus1)) + (pos2 / (2.0 * M_PI) - GetI2Cposition(moteus2));

  if (offset2 > 1.0) {
    offset2 -= 1.0;
  } else if (offset2 < -1.0) {
    offset2 += 1.0;
  }

  double pos_2 = GetSPIposition(moteus2) + offset2;

  position_cmd1.position = pos_1;
  position_cmd2.position = pos_2;
  // setDoublePosition(pos_1, pos_2, 0.001);
  moteus1.SetPosition(position_cmd1, &position_fmt);
  moteus2.SetPosition(position_cmd2, &position_fmt);
}


void setDoublePosition(double pos1, double pos2, double period_s) {

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
    if (count == 0 && got_result1 && got_result2 && moteus1.last_result().values.trajectory_complete && moteus2.last_result().values.trajectory_complete) {
      return;
    }

    delay(static_cast<unsigned long>(period_s * 1000));
  }
}

void run() {
  moteus1.Poll();
  moteus2.Poll();

  double m13_pos_cmd = constrain(position_cmd1.position, -0.25, 0.25);
  double m14_pos_cmd = constrain(position_cmd2.position, -0.25, 0.25);

  // double m13_pos_cmd = deg2rad(pos1);
  // double m14_pos_cmd = deg2rad(pos2);

  auto offset = (m13_pos_cmd / (2.0 * M_PI) - GetI2Cposition(moteus1)) + (m14_pos_cmd / (2.0 * M_PI) - GetI2Cposition(moteus2));
  if (offset > 1.0) {
    offset -= 1.0;
  } else if (offset < -1.0) {
    offset += 1.0;
  }
  position_cmd1.position = GetSPIposition(moteus1) + offset;


  auto offset2 = (m13_pos_cmd / (2.0 * M_PI) - GetI2Cposition(moteus1)) - (m14_pos_cmd / (2.0 * M_PI) - GetI2Cposition(moteus2));

  if (offset2 > 1.0) {
    offset2 -= 1.0;
  } else if (offset2 < -1.0) {
    offset2 += 1.0;
  }

  position_cmd2.position = GetSPIposition(moteus2) + offset2;

  setDoublePosition(position_cmd1.position, position_cmd2.position, 0.02);
  // moteus1.SetPosition(position_cmd1, &position_fmt);
  // moteus2.SetPosition(position_cmd2, &position_fmt);
}


byte wait_for_esp_response(int timeout, char* term = OKrn) {
  unsigned long t = millis();
  bool found = false;
  int i = 0;
  int len = strlen(term);
  // wait for at most timeout milliseconds
  // or if OK\r\n is found
  while (millis() < t + timeout) {
    if (HWSERIAL.available()) {
      buffer[i++] = HWSERIAL.read();
      if (i >= len) {
        if (strncmp(buffer + i - len, term, len) == 0) {
          found = true;
          break;
        }
      }
    }
  }
  buffer[i] = 0;
  Serial.print(buffer);
  return found;
}

bool read_till_eol() {
  static int i = 0;
  while (HWSERIAL.available()) {
    buffer[i++] = HWSERIAL.read();
    if (i == BUFFER_SIZE) i = 0;
    if (i > 1 && buffer[i - 2] == 13 && buffer[i - 1] == 10) {
      buffer[i] = 0;
      i = 0;
      Serial.print(buffer);
      return true;
    }
  }
  return false;
}

void setupWiFi() {

  HWSERIAL.println("ATE1");
  wait_for_esp_response(10000);
  HWSERIAL.println("AT+CIPCLOSE=4");
  wait_for_esp_response(10000);
  HWSERIAL.println("AT+CIPMUX=1");
  wait_for_esp_response(10000);
  HWSERIAL.println("AT+CIPSTART=4,\"UDP\",\"192.168.0.22\",9999,9998,0");
  wait_for_esp_response(10000);

  // HWSERIAL.println("AT+CIPSEND=4,7");
  // wait_for_esp_response(5000);
  // HWSERIAL.write("UDPtest");
  // wait_for_esp_response(5000);

  // Serial.println("ready");
}

void sendData(int motorId, int state, float pos, float i2c_pos, float velocity, float torque, float q_current, float d_current, float voltage, float temperature) {

  // Serial.println("send!");
  pos = floor(pos * 1000) / 1000;
  i2c_pos = floor(i2c_pos * 1000) / 1000;
  velocity = floor(velocity * 1000) / 1000;
  torque = floor(torque * 1000) / 1000;
  q_current = floor(q_current * 1000) / 1000;
  d_current = floor(d_current * 1000) / 1000;
  voltage = floor(voltage * 1000) / 1000;
  temperature = floor(temperature * 1000) / 1000;

  uint8_t bytes[8] = { 255, 255, 255, 255, motorId, 0, 0, state };                                   // 바이트 데이터
  float floats[8] = { pos, i2c_pos, velocity, torque, q_current, d_current, voltage, temperature };  // 실수 데이터

  int length = 8 + 8 * sizeof(float);
  unsigned char buffer_data[length];  // 바이트 배열 준비
  // 바이트 데이터 복사
  memcpy(buffer_data, bytes, 8);

  // 실수 데이터 복사, 빅 엔디안 변환 필요
  for (int i = 0; i < 8; i++) {
    uint32_t* floatAsInt = (uint32_t*)&floats[i];  // float를 uint32_t로 취급
    uint32_t bigEndianValue = htonl(floatAsInt);   // 호스트 엔디안을 네트워크 엔디안(빅 엔디안)으로 변환
    memcpy(buffer_data + 8 + i * sizeof(float), &bigEndianValue, sizeof(float));
  }
  // for(int i=0; i<length ; i++){

  //   Serial.print(buffer_data[i]);
  //   Serial.print(" ");
  // }
  //   Serial.println();
  HWSERIAL.write(buffer_data, sizeof(buffer_data));
}

uint32_t htonl(uint32_t* hostlong) {
  uint8_t* s = (uint8_t*)hostlong;
  return (uint32_t)(s[0] << 24) | (s[1] << 16) | (s[2] << 8) | s[3];
}