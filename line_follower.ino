const int NUM_SENSORS = 5;
const int sensorPins[NUM_SENSORS] = {A0, A1, A2, A3, A4};

const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int ENB = 9;
const int IN3 = 10;
const int IN4 = 11;

int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorVal[NUM_SENSORS];

float Kp = 0.18f;
float Ki = 0.0f;
float Kd = 0.05f;
float integral = 0;
float prevError = 0;

int baseSpeed = 140;
int maxSpeed  = 220;
int lastPosition = 0;
bool lineLost = false;
unsigned long lostSince = 0;

void setMotors(int left, int right) {
  if (left >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    left = -left;
  }
  if (right >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    right = -right;
  }
  analogWrite(ENA, constrain(left, 0, maxSpeed));
  analogWrite(ENB, constrain(right, 0, maxSpeed));
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void calibrate() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
  Serial.println(F("Calibrating - sweep sensors over black and white"));
  unsigned long start = millis();
  while (millis() - start < 4000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int v = analogRead(sensorPins[i]);
      if (v < sensorMin[i]) sensorMin[i] = v;
      if (v > sensorMax[i]) sensorMax[i] = v;
    }
  }
  Serial.println(F("Calibration done"));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("S")); Serial.print(i);
    Serial.print(F(" min=")); Serial.print(sensorMin[i]);
    Serial.print(F(" max=")); Serial.println(sensorMax[i]);
  }
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int v = analogRead(sensorPins[i]);
    int range = sensorMax[i] - sensorMin[i];
    if (range < 10) range = 10;
    sensorVal[i] = constrain((int)(((long)(v - sensorMin[i]) * 1000L) / range), 0, 1000);
  }
}

int getPosition() {
  long weighted = 0;
  long total = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    weighted += (long)sensorVal[i] * (i * 1000);
    total += sensorVal[i];
  }
  if (total < 120) {
    lineLost = true;
    if (lostSince == 0) lostSince = millis();
    return lastPosition; // hold last known
  }
  lineLost = false;
  lostSince = 0;
  int pos = (int)(weighted / total) - 2000;
  lastPosition = pos;
  return pos;
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'p' || c == 'P') {
    float v = Serial.parseFloat();
    if (v >= 0) { Kp = v; Serial.print(F("Kp=")); Serial.println(Kp); }
  } else if (c == 'i' || c == 'I') {
    float v = Serial.parseFloat();
    if (v >= 0) { Ki = v; integral = 0; Serial.print(F("Ki=")); Serial.println(Ki); }
  } else if (c == 'd' || c == 'D') {
    float v = Serial.parseFloat();
    if (v >= 0) { Kd = v; Serial.print(F("Kd=")); Serial.println(Kd); }
  } else if (c == 'b' || c == 'B') {
    int v = Serial.parseInt();
    if (v > 0 && v < 255) { baseSpeed = v; Serial.print(F("base=")); Serial.println(baseSpeed); }
  }
  while (Serial.available()) Serial.read();
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  Serial.begin(9600);
  calibrate();
  Serial.println(F("Line follower running. Commands: p/i/d/b for gains and base speed"));
}

void loop() {
  handleSerial();
  readSensors();
  int error = getPosition();

  if (lineLost && (millis() - lostSince > 800)) {
    // recovery: spin slowly toward last known side
    if (lastPosition > 0) setMotors(baseSpeed / 2, -baseSpeed / 3);
    else                  setMotors(-baseSpeed / 3, baseSpeed / 2);
    Serial.println(F("LINE LOST - recovering"));
    delay(30);
    return;
  }

  float dt = 0.02f;
  float pTerm = Kp * error;
  integral += error * dt;
  if (integral > 500) integral = 500;
  if (integral < -500) integral = -500;
  float iTerm = Ki * integral;
  float dTerm = Kd * (error - prevError) / dt;
  prevError = error;

  int correction = (int)(pTerm + iTerm + dTerm);
  int leftSpeed  = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  setMotors(leftSpeed, rightSpeed);

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(F("err=")); Serial.print(error);
    Serial.print(F(" L=")); Serial.print(leftSpeed);
    Serial.print(F(" R=")); Serial.println(rightSpeed);
  }

  delay(20);
}
