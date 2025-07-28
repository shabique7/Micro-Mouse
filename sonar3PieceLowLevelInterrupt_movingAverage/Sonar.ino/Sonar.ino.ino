// -------------------- Sonar Buffers --------------------
float buffer2[5] = {0}; int index2 = 0; float sum2 = 0;
float buffer3[5] = {0}; int index3 = 0; float sum3 = 0;
float buffer4[5] = {0}; int index4 = 0; float sum4 = 0;

float filtered_distance_2 = 0;
float filtered_distance_3 = 0;
float filtered_distance_4 = 0;

// -------------------- Pin Definitions --------------------
const int trigEchoPin2 = 4;  // Front - INT0
const int trigEchoPin3 = 3;  // Right - INT1
const int trigEchoPin4 = 2;  // Left  - PCINT20

// Motor direction pins
const int RIGHT_BACKWARD = 5;
const int RIGHT_FORWARD  = 6;
const int LEFT_FORWARD   = 9;
const int LEFT_BACKWARD  = 10;

// Motor enable (PWM) pins
const int ENA = 11;  // Right motor speed
const int ENB = 3;   // Left motor speed

//Motor speed (0–255)
int motorSpeed = 100;  // Reduced speed

const float OBSTACLE_THRESHOLD_CM = 10.0;  // Updated threshold to 10 cm

// -------------------- Sonar State --------------------
volatile bool readingReady2 = 0;
volatile float distance_cm2 = 0.0;
volatile unsigned long echoStartTime2 = 0;
volatile bool lastEchoState2 = LOW;

volatile bool readingReady3 = 0;
volatile float distance_cm3 = 0.0;
volatile unsigned long echoStartTime3 = 0;
volatile bool lastEchoState3 = LOW;

volatile bool readingReady4 = 0;
volatile float distance_cm4 = 0.0;
volatile unsigned long echoStartTime4 = 0;
volatile bool lastEchoState4 = LOW;

// -------------------- Interrupts --------------------
ISR(INT0_vect) {  // Pin 2 - Front
  bool state = digitalRead(trigEchoPin2);
  if (state && !lastEchoState2) {
    echoStartTime2 = micros();
  } else if (!state && lastEchoState2) {
    unsigned long duration = micros() - echoStartTime2;
    distance_cm2 = duration * 0.0343 / 2.0;
    readingReady2 = 1;
  }
  lastEchoState2 = state;
}

ISR(INT1_vect) {  // Pin 3 - Right
  bool state = digitalRead(trigEchoPin3);
  if (state && !lastEchoState3) {
    echoStartTime3 = micros();
  } else if (!state && lastEchoState3) {
    unsigned long duration = micros() - echoStartTime3;
    distance_cm3 = duration * 0.0343 / 2.0;
    readingReady3 = 1;
  }
  lastEchoState3 = state;
}

ISR(PCINT2_vect) {  // Pin 4 - Left
  bool state = digitalRead(trigEchoPin4);
  if (state && !lastEchoState4) {
    echoStartTime4 = micros();
  } else if (!state && lastEchoState4) {
    unsigned long duration = micros() - echoStartTime4;
    distance_cm4 = duration * 0.0343 / 2.0;
    readingReady4 = 1;
  }
  lastEchoState4 = state;
}

// -------------------- Sonar Triggering --------------------
void applyTriggerWhenReady() {
  if (readingReady2 == 0) {
    EIMSK &= ~(1 << INT0);
    triggerSonar(trigEchoPin2);
    EIMSK |= (1 << INT0);
  }

  if (readingReady3 == 0) {
    EIMSK &= ~(1 << INT1);
    triggerSonar(trigEchoPin3);
    EIMSK |= (1 << INT1);
  }

  if (readingReady4 == 0) {
    PCICR &= ~(1 << PCIE2);
    triggerSonar(trigEchoPin4);
    PCICR |= (1 << PCIE2);
  }
}

void triggerSonar(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
}

// -------------------- Moving Average --------------------
void updateMovingAverage(float newValue, float* buffer, int index, float &sum) {
  sum -= buffer[index];
  buffer[index] = newValue;
  sum += newValue;
}

void filter_distance() {
  if (readingReady2) {
    readingReady2 = 0;
    updateMovingAverage(distance_cm2, buffer2, index2, sum2);
    index2 = (index2 + 1) % 5;
    filtered_distance_2 = sum2 / 5.0;
  }

  if (readingReady3) {
    readingReady3 = 0;
    updateMovingAverage(distance_cm3, buffer3, index3, sum3);
    index3 = (index3 + 1) % 5;
    filtered_distance_3 = sum3 / 5.0;
  }

  if (readingReady4) {
    readingReady4 = 0;
    updateMovingAverage(distance_cm4, buffer4, index4, sum4);
    index4 = (index4 + 1) % 5;
    filtered_distance_4 = sum4 / 5.0;
  }
}

// -------------------- Setup Sonar --------------------
void setupSonar() {
  pinMode(trigEchoPin2, INPUT);
  pinMode(trigEchoPin3, INPUT);
  pinMode(trigEchoPin4, INPUT);

  // INT0 (pin 2)
  EICRA |= (1 << ISC00);  EIMSK |= (1 << INT0);

  // INT1 (pin 3)
  EICRA |= (1 << ISC10);  EIMSK |= (1 << INT1);

  // PCINT for pin 4
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);
}

// -------------------- Motor Control --------------------
void stopMotors() {
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(RIGHT_BACKWARD, LOW);
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(LEFT_BACKWARD, LOW);
}

void moveForward() {
  digitalWrite(RIGHT_FORWARD, HIGH);
  digitalWrite(RIGHT_BACKWARD, LOW);
  digitalWrite(LEFT_FORWARD, HIGH);
  digitalWrite(LEFT_BACKWARD, LOW);
}

bool obstacleDetected() {
  return (filtered_distance_2 < OBSTACLE_THRESHOLD_CM ||
          filtered_distance_3 < OBSTACLE_THRESHOLD_CM ||
          filtered_distance_4 < OBSTACLE_THRESHOLD_CM);
}

// -------------------- Main Program --------------------
uint32_t loopTime = 0;

void setup() {
  Serial.begin(9600);
  setupSonar();

  // Motor pin setup
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_BACKWARD, OUTPUT);
  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_BACKWARD, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  analogWrite(ENA, motorSpeed);  // Set motor speed (PWM)
  analogWrite(ENB, motorSpeed);
  stopMotors();

  loopTime = micros();
}

void loop() {
  applyTriggerWhenReady();
  filter_distance();

  // Print filtered sonar readings
  Serial.print("Right: ");
  Serial.print(filtered_distance_2);
  Serial.print(" cm | Left: ");
  Serial.print(filtered_distance_3);
  Serial.print(" cm | Front: ");
  Serial.println(filtered_distance_4);

  // Obstacle avoidance based on 10 cm threshold
  if (obstacleDetected()) {
    stopMotors();
  } else {
    moveForward();
  }

  // Maintain 200 Hz loop
  loopTime += 5000;
  while ((int32_t)(micros() - loopTime) < 0);
}
