// Moving average buffers and counters for pin 2
float buffer2[5] = {0};
int index2 = 0;
float sum2 = 0;

// Moving average buffers and counters for pin 3
float buffer3[5] = {0};
int index3 = 0;
float sum3 = 0;

// Moving average buffers and counters for pin 4
float buffer4[5] = {0};
int index4 = 0;
float sum4 = 0;


// Pin Definitions
const int trigEchoPin2 = 2;  // INT0
const int trigEchoPin3 = 3;  // INT1
const int trigEchoPin4 = 4;  // PCINT20

// Motor direction pins
const int RIGHT_BACKWARD = 5;
const int RIGHT_FORWARD  = 6;
const int LEFT_FORWARD   = 9;
const int LEFT_BACKWARD  = 10;

// Motor speed (0–255)
int motorSpeed = 100;  // Reduced speed

const float OBSTACLE_THRESHOLD_CM = 10.0;  // Updated threshold to 10 cm

// Variables for Pin 2
volatile bool readingReady2 = 0;
volatile float distance_cm2 = 0.0;
volatile unsigned long echoStartTime2 = 0;
volatile bool lastEchoState2 = LOW;

// Variables for Pin 3
volatile bool readingReady3 = 0;
volatile float distance_cm3 = 0.0;
volatile unsigned long echoStartTime3 = 0;
volatile bool lastEchoState3 = LOW;

// Variables for Pin 4
volatile bool readingReady4 = 0;
volatile float distance_cm4 = 0.0;
volatile unsigned long echoStartTime4 = 0;
volatile bool lastEchoState4 = LOW;

// ---------- Interrupt Service Routines ------------

ISR(INT0_vect) {  // Pin 2
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

ISR(INT1_vect) {  // Pin 3
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

ISR(PCINT2_vect) {  // Pin 4
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







// ---------- Trigger Pulse Function ------------

void applyTriggerWhenReady(){
// Sensor 1 - Pin 2
  if (readingReady2 == 0) {
    EIMSK &= ~(1 << INT0);  // Disable INT0
    triggerSonar(trigEchoPin2);
    EIMSK |= (1 << INT0);   // Re-enable INT0
  }

    // Sensor 2 - Pin 3
  if (readingReady3 == 0) {
    EIMSK &= ~(1 << INT1);  // Disable INT1
    triggerSonar(trigEchoPin3);
    EIMSK |= (1 << INT1);   // Re-enable INT1
  }

  // Sensor 3 - Pin 4
  if (readingReady4 == 0) {
    PCICR &= ~(1 << PCIE2);  // Disable PCINT for PORTD
    triggerSonar(trigEchoPin4);
    PCICR |= (1 << PCIE2);   // Re-enable PCINT
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



void updateMovingAverage(float newValue, float* buffer, int index, float &sum) {
  sum -= buffer[index];
  buffer[index] = newValue;
  sum += newValue;
}


void filter_distance(){
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


void setupSonar(){
    // Setup pins as input
  pinMode(trigEchoPin2, INPUT);
  pinMode(trigEchoPin3, INPUT);
  pinMode(trigEchoPin4, INPUT);

  // Enable external interrupt for pin 2 (INT0)
  EICRA |= (1 << ISC00);  // Interrupt on any logic change
  EIMSK |= (1 << INT0);   // Enable INT0

  // Enable external interrupt for pin 3 (INT1)
  EICRA |= (1 << ISC10);  // Interrupt on any logic change
  EIMSK |= (1 << INT1);   // Enable INT1

  // Enable Pin Change Interrupt for pin 4 (PCINT20 = PD4)
  PCICR |= (1 << PCIE2);     // Enable PCINT[23:16] (PORTD)
  PCMSK2 |= (1 << PCINT20);  // Enable interrupt for PD4
}