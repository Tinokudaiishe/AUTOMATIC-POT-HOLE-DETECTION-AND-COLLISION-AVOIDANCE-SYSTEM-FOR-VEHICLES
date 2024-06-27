#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>

// Define the pins for the ultrasonic sensor
int trigPin = A1;
int echoPin = A2;

// Define the pins for the vibration sensor
int vibrationPin = A0;

// Define the pins for the buzzer and LED
int buzzerPin = 9;
int ledPin = 10;

// Define the GPS object
TinyGPSPlus gps;

LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  // Set the pin modes
  lcd.begin(20, 4);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(vibrationPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  lcd.print("Pothole Detector");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");
  delay(700);

  // Start the serial communication
  Serial.begin(9600);
}

void loop() {
  // Ultrasonic sensor
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) / 29.1;

  // Check if an object is detected
  if (distance < 20) {
    // Turn on the buzzer and LED
    digitalWrite(buzzerPin, HIGH);
    digitalWrite(ledPin, HIGH);
    Serial.println("Object detected!");
    lcd.clear();
    lcd.print("Obstacle ahead!");
    lcd.setCursor(0, 1);
    lcd.print("Slow down");
  } else {
    // Turn off the buzzer and LED
    digitalWrite(buzzerPin, LOW);
    digitalWrite(ledPin, LOW);
    lcd.clear();
    lcd.print("Drive safely!");
    delay(300);
    lcd.clear();
  }

  // Vibration sensor
  int vibrationValue = digitalRead(vibrationPin);

  // Check if vibration is detected
  if (vibrationValue == HIGH) {
    // Check if 'P' is received through the com port
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 'P') {
        // Get the current location from the GPS module
        while (Serial.available() > 0) {
          if (gps.encode(Serial.read())) {
            if (gps.location.isValid()) {
              float latitude = gps.location.lat();
              float longitude = gps.location.lng();

              // Turn on the buzzer and LED
              digitalWrite(buzzerPin, HIGH);
              digitalWrite(ledPin, HIGH);
              Serial.println("Pothole detected!");
              lcd.clear();
              lcd.print("Pothole detected!");
              lcd.setCursor(0, 1);
              lcd.print("Lat: ");
              lcd.print(latitude, 3);
              lcd.print(", Lon: ");
              lcd.print(longitude, 3);
              delay(1000);

              // Turn off the buzzer and LED
              digitalWrite(buzzerPin, LOW);
              digitalWrite(ledPin, LOW);
            }
          }
        }
      }
    }
  }

  // Delay for 100 milliseconds
  delay(100);
}
