uint32_t loopTime = 0;

float filtered_distance_2, filtered_distance_3, filtered_distance_4;

void setup() {
  Serial.begin(9600);

  loopTime = micros();
}


void loop() {
  applyTriggerWhenReady();  // call your existing trigger function
  filter_distance();

// calculateError();
  Serial.print("Right (Pin 2): ");
  Serial.print(filtered_distance_2);
  Serial.print(" cm | Left (Pin 3): ");
  Serial.print(filtered_distance_3);
  Serial.print(" cm | Front (Pin 4): ");
  Serial.print(filtered_distance_4);
  Serial.println(" cm");

  // Obstacle avoidance based on 10 cm threshold
  if (obstacleDetected()) {
    stopMotors();
  } else {
    moveForward();
  }

// applyPID();
// motor(x,y);

  while (micros() - loopTime <= 5000);  // Ensure 200 Hz
  loopTime = micros();
}



                                    
