#include <esp32-hal-ledc.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>

// Pin Definitions
#define ESC_PIN1 12  // Pin สำหรับส่งสัญญาณ PWM ไปยัง ESC
#define ESC_PIN2 13  // Pin สำหรับส่งสัญญาณ PWM ไปยัง ESC
#define TOF_RX_PIN 16  // UART RX pin ของเซนเซอร์ TOF
#define TOF_TX_PIN 17  // UART TX pin ของเซนเซอร์ TOF

// PID Parameters
double setPoint = 100.0;  // ระยะทางที่ต้องการ (cm)
double currentDistance = 0.0;  // ระยะทางที่อ่านได้จากเซนเซอร์ TOF
double outputPWM = 0.0;  // ค่า PWM สำหรับ ESC
double Kp = 2.0, Ki = 0.5, Kd = 1.0;  // ค่า PID

// PID Controller
PID myPID(&currentDistance, &outputPWM, &setPoint, Kp, Ki, Kd, DIRECT);

// ESC Configuration
const int ESC_MIN = 1000;  // Minimum PWM (μs)
const int ESC_MAX = 2000;  // Maximum PWM (μs)

// UART สำหรับ TOF Sensor
HardwareSerial TOFSerial(1);

void writeESC(int pwmValue);
double readTOFDistance();

void setup() {
  // ตั้งค่า UART สำหรับ TOF
  TOFSerial.begin(115200, SERIAL_8N1, TOF_RX_PIN, TOF_TX_PIN);

  // ตั้งค่า ESC Pin
  pinMode(ESC_PIN1, OUTPUT);
  pinMode(ESC_PIN2,OUTPUT);
  ledcSetup(0, 50, 16); // ใช้ PWM ช่อง 0 ที่ 50Hz
  ledcAttachPin(ESC_PIN1, 0);
  ledcAttachPin(ESC_PIN2, 0);

  // เริ่มต้น PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(ESC_MIN, ESC_MAX);

  // เริ่มต้น ESC
  writeESC(ESC_MIN);  // ส่งค่าต่ำสุดเพื่อปลดล็อค ESC
  delay(2000);        // รอ ESC ทำงาน
}

void loop() {
  // อ่านค่าระยะทางจากเซนเซอร์ TOF
  currentDistance = readTOFDistance();

  // คำนวณ PID
  myPID.Compute();

  // ส่งค่าที่คำนวณได้ไปยัง ESC
  writeESC((int)outputPWM);

  // Debug
  Serial.print("Distance: ");
  Serial.print(currentDistance);
  Serial.print(" cm, Output PWM: ");
  Serial.println(outputPWM);

  delay(50);  // หน่วงเวลาเล็กน้อย
}

// ฟังก์ชันสำหรับส่งสัญญาณ PWM ไปยัง ESC
void writeESC(int pwmValue) {
  pwmValue = constrain(pwmValue, ESC_MIN, ESC_MAX);
  ledcWrite(0, map(pwmValue, 1000, 2000, 0, 2000)); // 16-bit resolution
}

// ฟังก์ชันอ่านค่าระยะทางจาก TOF Sensor
double readTOFDistance() {
  if (TOFSerial.available()) {
    // อ่านข้อมูลจาก UART ของเซนเซอร์ TOF (ปรับตามโปรโตคอลของเซนเซอร์ TOF400F)
    String data = TOFSerial.readStringUntil('\n');
    return data.toFloat();  // แปลงข้อมูลเป็นตัวเลข
  }
  return currentDistance;  // ถ้าไม่มีข้อมูลใหม่ ให้ใช้ค่าปัจจุบัน
}
