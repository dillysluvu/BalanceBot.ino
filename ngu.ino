//Thư viện PID
#include <PID_v1.h>

//Thư viện cho MPU6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define INTERRUPT_PIN 2  // Chân ngắt của MPU6050

// Biến trạng thái cho MPU
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Biến lưu trữ dữ liệu định hướng
Quaternion q;
VectorFloat gravity;
float ypr[3];
VectorInt16 gy;

// Hàm ngắt khi có dữ liệu mới từ MPU6050
volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

// Định nghĩa giới hạn PID và thời gian lấy mẫu
#define PID_MIN_LIMIT -255
#define PID_MAX_LIMIT 255
#define PID_SAMPLE_TIME_IN_MILLI 10

// Giá trị offset để robot đứng thẳng
#define SETPOINT_PITCH_ANGLE_OFFSET -2.2

// Tốc độ tối thiểu cho động cơ
#define MIN_ABSOLUTE_SPEED 0

double setpointPitchAngle = SETPOINT_PITCH_ANGLE_OFFSET;
double pitchGyroAngle = 0;
double pitchPIDOutput = 0;

double setpointYawRate = 0;
double yawGyroRate = 0;
double yawPIDOutput = 0;

// Hằng số PID
#define PID_PITCH_KP 20
#define PID_PITCH_KI 70
#define PID_PITCH_KD 0.5

#define PID_YAW_KP 0.5
#define PID_YAW_KI 0.5
#define PID_YAW_KD 0

PID pitchPID(&pitchGyroAngle, &pitchPIDOutput, &setpointPitchAngle, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, DIRECT);
PID yawPID(&yawGyroRate, &yawPIDOutput, &setpointYawRate, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, DIRECT);

// Chân điều khiển động cơ
int enableMotor1 = 9;
int motor1Pin1 = 5;
int motor1Pin2 = 6;

int motor2Pin1 = 7;
int motor2Pin2 = 8;
int enableMotor2 = 10;

// Hàm thiết lập PID
void setupPID() {
  pitchPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);

  yawPID.SetOutputLimits(PID_MIN_LIMIT, PID_MAX_LIMIT);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetSampleTime(PID_SAMPLE_TIME_IN_MILLI);
}

// Hàm thiết lập điều khiển động cơ
void setupMotors() {
  pinMode(enableMotor1, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(enableMotor2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  rotateMotor(0, 0);
}

// Hàm thiết lập MPU6050
void setupMPU() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // Tốc độ I2C 400kHz
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();

  // Thiết lập offset cho cảm biến
  mpu.setXAccelOffset(-122);
  mpu.setYAccelOffset(111);
  mpu.setZAccelOffset(800);
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-32);
  mpu.setZGyroOffset(38);

  // Kiểm tra kết nối cảm biến
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}

void setup() {
  setupMotors();   // Thiết lập điều khiển động cơ
  setupMPU();      // Thiết lập MPU6050
  setupPID();      // Thiết lập PID
}

void loop() {
  // Kiểm tra kết nối cảm biến
  if (!dmpReady) return;

  // Đọc dữ liệu từ FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);

    yawGyroRate = gy.z;                   // Tốc độ góc quay
    pitchGyroAngle = ypr[1] * 180 / M_PI; // Góc nghiêng

    pitchPID.Compute(true);
    yawPID.Compute(true);

    rotateMotor(pitchPIDOutput + yawPIDOutput, pitchPIDOutput - yawPIDOutput);

    // In ra dữ liệu debug (nếu cần)
    #ifdef PRINT_DEBUG_BUILD
      Serial.println("Góc nghiêng: ");
      Serial.println(pitchGyroAngle);
      Serial.println("Giá trị đặt: ");
      Serial.println(setpointPitchAngle);
      Serial.println("Đầu ra PID: ");
      Serial.println(pitchPIDOutput);
      delay(500);
    #endif
  }
}

// Hàm điều khiển động cơ
void rotateMotor(int speed1, int speed2) {
  // Thiết lập chiều quay cho động cơ 1
  if (speed1 < 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  } else if (speed1 >= 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  }

  // Thiết lập chiều quay cho động cơ 2
  if (speed2 < 0) {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else if (speed2 >= 0) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }

  // Đảm bảo tốc độ không dưới ngưỡng tối thiểu
  speed1 = abs(speed1) + MIN_ABSOLUTE_SPEED;
  speed2 = abs(speed2) + MIN_ABSOLUTE_SPEED;

  // Giới hạn tốc độ trong khoảng [MIN_ABSOLUTE_SPEED, 255]
  speed1 = constrain(speed1, MIN_ABSOLUTE_SPEED, 255);
  speed2 = constrain(speed2, MIN_ABSOLUTE_SPEED, 255);

  // Điều khiển tốc độ động cơ
  analogWrite(enableMotor1, speed1);
  analogWrite(enableMotor2, speed2);
}
