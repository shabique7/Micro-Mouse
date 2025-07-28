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

// Main variables
uint32_t loopTime = 0;
float filtered_distance_2, filtered_distance_3, filtered_distance_4;

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

ISR(PCINT2_vect) {  // Pin 4 - Pin Change Interrupt for PORTD
  // Check if the interrupt is specifically for pin 4
  static bool pin4_last_state = LOW;
  bool current_state = digitalRead(trigEchoPin4);
  
  // Only process if pin 4 actually changed
  if (current_state != pin4_last_state) {
    if (current_state && !lastEchoState4) {
      echoStartTime4 = micros();
    } else if (!current_state && lastEchoState4) {
      unsigned long duration = micros() - echoStartTime4;
      distance_cm4 = duration * 0.0343 / 2.0;
      readingReady4 = 1;
    }
    lastEchoState4 = current_state;
    pin4_last_state = current_state;
  }
}

// ---------- Setup Function ------------

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
  PCMSK2 |= (1 << PCINT20);  // Enable interrupt for PD4 (pin 4)
}

void setup() {
  Serial.begin(9600);
  setupSonar();
  loopTime = micros();
}



// ---------- Main Loop ------------

void loop() {
  applyTriggerWhenReady();  // call your existing trigger function
  filter_distance();

  // calculateError();
  // applyPID();
  // motor(x,y);
  
  Serial.print(filtered_distance_2);
  Serial.print(" ");
  Serial.print(filtered_distance_3);
  Serial.print(" ");
  Serial.print(filtered_distance_4);
  Serial.println(" ");

  while (micros() - loopTime <= 5000);  // Ensure 200 Hz
  loopTime = micros();
}