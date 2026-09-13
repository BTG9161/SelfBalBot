#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "Adafruit_MPU6050.h"
#include <madgwickFilter.h>


#define AIN1  25
#define BIN1  26

#define AIN2  16
#define BIN2  17

#define PWMA 32
#define PWMB 23

#define CH_A 0
#define CH_B 1


Adafruit_MPU6050 mpu;
WiFiClient client;
WiFiServer server(8080);
sensors_event_t a, g, temp;

const char* ssid = "DRONE_WIFI";
const char* pass = "12345678";

bool armed = false;
bool Connected = false;

char c;

float ax, ay, az;
float gx, gy, gz;

int InputPitch;

float pitch;
float ratePitch;
float ErrorPitch;

float PRatePitch;
float IRatePitch;
float DRatePitch;

float PID_Return [] = {0, 0, 0};
float PrevErrorPitch;
float PrevItermPitch;


void PID(float Error, float P, float I, float D ,float PrevError, float PrevIterm, float dt){
  float Pterm = P * Error;
  
  float Iterm = PrevIterm + I * (Error + PrevError) * dt * 0.5f;
  Iterm = constrain(Iterm, -400, 400);

  float Dterm = -D * ratePitch;

  float PID_Output = Pterm + Iterm + Dterm;
  PID_Output = constrain(PID_Output, -400, 400);

  PID_Return[0] = PID_Output;
  PID_Return[1] = Error;
  PID_Return[2] = Iterm;
}

void setMotor(int in1, int in2, int channel, int speed) {
  if (speed >= 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(channel, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(channel, -speed);
  }
}

void mixMotors(int pitchPID) {
  int mA = pitchPID;
  int mB = pitchPID;

  // Normalize instead of brutal clipping
  int maxMotor = max(abs(mA), abs(mB));

  if (maxMotor > 255) {
    int excess = maxMotor - 255;
    mA -= excess;
    mB -= excess;
  }

  // Lower bound safety
  mA = constrain(mA, -255, 255);
  mB = constrain(mB, -255, 255);

  setMotor(AIN1, AIN2, CH_A, mA);
  setMotor(BIN1, BIN2, CH_B, mB);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire.setClock(400000);
  mpu.begin();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, pass);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  server.begin();

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  //pinMode(STBY, OUTPUT);
  //digitalWrite(STBY, HIGH); // Enable driver, can be later used for arming or disarming the quadcopter

  // PWM setup
  ledcSetup(CH_A, 20000, 8);
  ledcSetup(CH_B, 20000, 8);

  ledcAttachPin(PWMA, CH_A);
  ledcAttachPin(PWMB, CH_B);
}

void loop() {
  //Calculates dt
  static unsigned long lastTime;
  unsigned long now = micros();

  if (lastTime == 0) {
    lastTime = now;
    return;
  }
  
  float dt = (now - lastTime) / 1000000.0f;
  lastTime = now;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;


  //Reading data
  if (!client || !client.connected()) {
    client = server.available();
  }
  if (client && client.connected()) {
    Serial.println("Connection Successful");
    Connected = true;

    while (client.available()) {
      int val = client.read();
      
      if (val != -1) {
        c = (char)val;
        Serial.print(c);

        if (c == 'W') {
          InputPitch += 100;
        }
        else if (c == 'S') {
          InputPitch -= 100;
        }
        else if (c == 'X') {
          armed = true;
        }
        else if (c == 'Z') {
          armed = false;
        }
      }
    }
  }

  if (!armed || !Connected) {
    mixMotors(0);
    return;
  }
  if (c == 'Q'){
    client.stop();
  }
  
  //Mapping and converting data to desired formats
  pitch = map(InputPitch, 0, 1023, -20, 20);

  mpu.getEvent(&a, &g, &temp);
  
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;

  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;

  ratePitch = gy * 57.2958;
    
  // Update Madgwick filter
  imu_filter(ax, ay, az, gx, gy, gz, dt);

  float m_roll = 0;
  float m_pitch = 0;
  float m_yaw = 0;

  eulerAngles(q_est, &m_roll, &m_pitch);
  
  ErrorPitch = pitch - m_pitch;
  
  PID(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorPitch, PrevItermPitch, dt);
  float pitchPID = PID_Return[0];
  PrevErrorPitch = PID_Return[1];
  PrevItermPitch = PID_Return[2];

  mixMotors((int)pitchPID);

}

