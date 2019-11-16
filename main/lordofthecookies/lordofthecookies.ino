//
// Lord Of The Cookies Arduino Sketch
//
// Author: Alan Zhao
// Date: 11/10/2019
//

#define pressed LOW
#define unpressed HIGH

#define dirLeft LOW
#define dirRight HIGH

//
// Start pin definitions
//

// Slider pins
const int sliderStepPin = 2;
const int sliderDirPin = 3;
const int sliderEnablePin = 4;

// Actuator pins
const int actuatorEnable = 5;
const int actuatorIn1 = 6;
const int actuatorIn2 = 7;

// LED trigger pin
const int ledTriggerPin = 8;

// Pin #9 is reserved at the moment

// Slider limit switches pins
const int sliderRightPin = 10;
const int sliderLeftPin = 11;
const int sliderHomePin = sliderLeftPin;

// Buzzer pin
const int buzzer = 12;

// Push buttons
int buttonPins[] = { A1, A5, A4, A3, A2, A0 };
int numberOfButtonPins = sizeof(buttonPins) / sizeof(int);



//
// Other configurations
//

// Pulse/rev = 400 set on the driver = motorStepsPerRev x microSteps
const int motorStepsPerRev = 200;
const int microSteps = 2;

// The lower the value(less delay), the faster motor goes
const int sliderSpeed = 300;
const int speedFast = sliderSpeed;

// How many turns per button
float sliderTurns[] = { 5.5, 14.3, 23, 31.5, 40.1, 48.8 };

// Run actuator for how long
float runActuatorSeconds = 2.38;


//
// Start functions
//

// Set slider direction
void setSliderDir(int direction) {
  digitalWrite(sliderDirPin, direction);
}

// Run the slider motor with default speed CONTINOUSLY
void runSlider() {
  runSlider(speedFast);
}

// Run the slider motor with specified speed CONTINOUSLY
void runSlider(int speed) {
  digitalWrite(sliderStepPin, HIGH);
  delayMicroseconds(speed);
  digitalWrite(sliderStepPin, LOW);
  delayMicroseconds(speed);
}

// Pause the slider
void stopSlider() {
  digitalWrite(sliderStepPin, LOW);
}

// Run the slider motor for specified revolutions then STOP
void runSliderRevs(float revs) {
  float steps = round(motorStepsPerRev * microSteps * revs);
  for (long x = 0; x < steps; x++) {
    runSlider();
  }
  stopSlider();
}

// Run the slider motor for specified revolutions, stop if end is reached
void runSliderRevsAndCheckEnd(float revs) {
  float steps = round(motorStepsPerRev * microSteps * revs);
  for (long x = 0; x < steps; x++) {
    if (digitalRead(sliderRightPin) == pressed) {
      stopSlider();
      break;
    }
    runSlider();
  }
  stopSlider();
}

// Run the slider motor for specified revolutions and speed then STOP
void runSliderRevs(float revs, int speed) {
  float steps = round(motorStepsPerRev * microSteps * revs);
  for (long x = 0; x < steps; x++) {
    runSlider(speed);
  }
  stopSlider();
}

// Run slider in specified direction and revolutions
void runSliderDirAndRevs(int dir, int revs) {
  setSliderDir(dir);
  runSliderRevs(revs);
}

// Run slider away from HOME position
void sliderLeaveHome() {
  setSliderDir(dirRight);
  runSlider(speedFast);
}

// Run slider toward HOME position
void sliderComeHome() {
  setSliderDir(dirLeft);
  runSlider(speedFast);
}

// Stop actuator
void stopActuator() {
  analogWrite(actuatorEnable, 0); // speed 0-255
  digitalWrite(actuatorIn1, LOW);
  digitalWrite(actuatorIn2, LOW);
}

// Push actuator
void pushActuator() {
  analogWrite(actuatorEnable, 255); // speed 0-255
  digitalWrite(actuatorIn1, LOW);
  digitalWrite(actuatorIn2, HIGH);
}

// Pull actuator
void pullActuator() {
  analogWrite(actuatorEnable, 255); // speed 0-255
  digitalWrite(actuatorIn1, HIGH);
  digitalWrite(actuatorIn2, LOW);
}

// Push actuator for specified seconds and pull back for the same amount of time
void runActuator(float seconds) {
  pushActuator();
  delay(seconds * 1000);
  stopActuator();
  delay(2000);
  pullActuator();
  delay(seconds * 1000);
}

// Make beep sound for specified counts
void beep(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    delay(100);
  }
}

void setup() {

  // Start serial connection
  Serial.begin(9600);

  // Initialize slider pins
  pinMode(sliderStepPin, OUTPUT);
  pinMode(sliderDirPin, OUTPUT);
  pinMode(sliderEnablePin, OUTPUT);

  // Initialize actuator pins
  pinMode(actuatorEnable, OUTPUT);
  pinMode(actuatorIn1, OUTPUT);
  pinMode(actuatorIn2, OUTPUT);

  // Initialize LED trigger pin
  pinMode(ledTriggerPin, OUTPUT);

  // Initialize slider limit switch pins
  // Not connected = HIGH
  // NO pin = HIGH (when not pressed), LOW (when pressed)
  // NC pin = LOW (when not pressed), HIGH (when pressed)
  pinMode(sliderLeftPin, INPUT);
  pinMode(sliderRightPin, INPUT);

  // Initialize buzzer pin
  pinMode(buzzer, OUTPUT);

  // Initialize push button pins
  for ( int i = 0; i < numberOfButtonPins; i++) {
    int buttonPin = buttonPins[i];
    Serial.println((String)"Initializing button pin: " + (i + 1));
    pinMode(buttonPin, INPUT);
  }

  // Make sure actuator is pulled back
  pullActuator();

  // Move away a little
  setSliderDir(dirRight);
  runSliderRevs(0.1);

  // Start homing precedure at start up
  while (digitalRead(sliderHomePin) == unpressed) {
    sliderComeHome();
  }

  // Home now, move away a little
  while (digitalRead(sliderHomePin) == pressed) {
    sliderLeaveHome();
  }

  // Add a little space
  setSliderDir(dirRight);
  runSliderRevs(0.5);

  beep(2);
  Serial.write("Starting...");

  // Disable stepper
  digitalWrite(sliderEnablePin, HIGH);

  // LED on but not flash
  digitalWrite(ledTriggerPin, HIGH);
}

void loop() {

  bool restart = false;

  //Serial.println((String)"Slider left: " + digitalRead(sliderLeftPin));
  //Serial.println((String)"Slider right: " + digitalRead(sliderRightPin));

  for ( int i = 0; i < numberOfButtonPins; i++) {
    int buttonPin = buttonPins[i];

    while (digitalRead(buttonPin) == pressed) {

      beep(i + 1);

      //beep(1);

      float sliderTurn = sliderTurns[i];

      Serial.println((String)"Button pushed: " + (i + 1));
      Serial.println((String)"Turns: " + sliderTurn);

      // Flash LED
      digitalWrite(ledTriggerPin, LOW);

      // Enable stepper
      digitalWrite(sliderEnablePin, LOW);

      // Set slider direction
      setSliderDir(dirRight);

      // Run slider
      //Serial.println((String)"Running " + sliderTurn);
      runSliderRevsAndCheckEnd(sliderTurn);

      // Run actuator
      runActuator(runActuatorSeconds);

      // Bring slider back
      while (digitalRead(sliderHomePin) == unpressed) {
        sliderComeHome();
      }

      // Home now, move away a little
      while (digitalRead(sliderHomePin) == pressed) {
        sliderLeaveHome();
      }

      // Add a little space
      setSliderDir(dirRight);
      runSliderRevs(0.5);

      // Disable stepper
      digitalWrite(sliderEnablePin, HIGH);

      // Stop LED
      digitalWrite(ledTriggerPin, HIGH);

      restart = true;
    }

    if (restart) {
      break;
    }
  }
}
