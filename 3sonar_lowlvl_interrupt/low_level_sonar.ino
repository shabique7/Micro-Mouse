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

// ---------- Moving Average Filter Functions ------------

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