// Define Pins
#define ENCODER_PINA 2
#define ENCODER_PINB 3
const int M0_PHASE = 4;
#define M0_PHASE 6
#define M0_PWM 5

#define GEAR_RATIO 30
#define PULSES_PER_REVOLUTION 12
#define MAX_PWM 255
#define MAX_SENSOR 1023
#define MAX_RPM 300
#define Kp 1.5
#define Ki 1.5
#define Kd 0
#define A 0.1

volatile int encoderCounts = 0;
unsigned long prevTime = 0;
float prevRotations = 0;
float integral = 0;
float prevVelocity = 0;
float prevError = 0;

// Encoder ISR functions
void encoderA();
void encoderB();

void setup() {
  // Initialize serial communication at 115200 bits per second
  Serial.begin(115200);

  // Initialize encoder, attach ISR functions
  pinMode(ENCODER_PINA, INPUT);
  pinMode(ENCODER_PINB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINA), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINB), encoderB, CHANGE);
}

void loop() {
  int sensorValue = analogRead(A0);
  float targetVelocity = normalve(sensorValue, 0, MAX_SENSOR, -MAX_RPM, MAX_RPM);

  unsigned long currentTime = millis();
  unsigned long deltaTimeMs = currentTime - prevTime;
  if (deltaTimeMs < 10)
    return;
  prevTime = currentTime;
  float deltaTimeSec = (float)deltaTimeMs / 1000;

  // Convert encoder position to wheel rotations
  float rotations = (float)encoderCounts / (PULSES_PER_REVOLUTION * GEAR_RATIO);
  float deltaRotations = rotations - prevRotations;
  prevRotations = rotations;

  // Calculate wheel velocity in RPM units
  float velocity = (float)(deltaRotations * 60) / deltaTimeSec;
  float filteredVelocity = A * velocity + (1 - A) * prevVelocity;
  prevVelocity = filteredVelocity;
  // PID controller
  float error = targetVelocity - filteredVelocity;
  integral += error * deltaTimeSec;
  float derivative = (error - prevError) / deltaTimeSec;
  prevError = error;
  float output = Kp * error + Ki * integral + Kd * derivative;
  int pwm = min(abs(output), MAX_PWM);
  if (output >= 0) {
    analogWrite(M0_PWM, pwm);
    analogWrite(M0_PHASE, 0);
  } else {
    analogWrite(M0_PWM, 0);
    analogWrite(M0_PHASE, pwm);
  }
  Serial.print(targetVelocity);
  Serial.print(", ");
  Serial.print(filteredVelocity);
  Serial.print(", ");
  Serial.print(error);
  Serial.print("\n");
}

void encoderA() {
  if (digitalRead(ENCODER_PINA) == HIGH) {
    digitalRead(ENCODER_PINB) ? encoderCounts++ : encoderCounts--;
  } else {
    digitalRead(ENCODER_PINB) ? encoderCounts-- : encoderCounts++;
  }
}

void encoderB() {
  if (digitalRead(ENCODER_PINB) == HIGH) {
    digitalRead(ENCODER_PINA) ? encoderCounts-- : encoderCounts++;
  } else {
    digitalRead(ENCODER_PINA) ? encoderCounts++ : encoderCounts--;
  }
}

float normalve(int value, int oldMin, int oldMax, int newMin, int newMax) {
  int oldRange = oldMax - oldMin;
  int newRange = newMax - newMin;
  float scaledValue = (float)(value - oldMin) / oldRange;
  float normalizedValue = (float)(scaledValue * newRange) + newMin;
  return normalizedValue;
}
