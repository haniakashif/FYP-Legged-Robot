#include <Servo.h>

Servo myServo;
const int servoPin = 6;
const int potPin   = A0; // analog input from pot wiper
int pos = 95;

const int adcMin = 75;     // ADC at 0°
const int adcMax = 387;    // ADC at 180°

// // Set these after calibration
// int minADC = 50;    // example, replace with measured min
// int maxADC = 980;   // example, replace with measured max
// int servoMinDeg = 0;
// int servoMaxDeg = 180;

void setup() {
  // put your setup code here, to run once:
  // myServo.attach(6);
    Serial.begin(9600);
    myServo.attach(servoPin);

  //   // Optionally move servo to center
  //   myServo.write(90);
  //   delay(500);
}

// int readFilteredADC(int pin, int samples=8) {
//   long sum=0;
//   for (int i=0;i<samples;i++){
//     sum += analogRead(pin);
//     delay(2);
//   }
//   return (int)(sum / samples);
// }

void loop() {
  // put your main code here, to run repeatedly:
  // myServo.write(pos);
  delay(500);
  int adc = analogRead(potPin);
  Serial.print(adc);  
  Serial.print("\n");
  delay(500);

  // myServo.write(90);
  // delay(1000);
  // adc = analogRead(potPin);
  // Serial.print(adc);
  // Serial.print("\n");
  // delay(1000);

  // myServo.write(180);
  // delay(1000);
  // adc = analogRead(potPin);
  // Serial.print(adc);
  // Serial.print("\n");
  // delay(1000);


  int angleMap = map(adc, adcMin, adcMax, 0, 180);
  Serial.print("servo is at angle (feedback):");
  Serial.print(angleMap);
  Serial.print("\n");


  // int vaar = (380-75)/180;
  // Serial.print("servo is at angle (feedback):"); 
  // Serial.print(vaar*(adc-75));
  // Serial.print("\n");
//   // clamp
//   adc = max(minADC, min(maxADC, adc));

//   // map to angle (float)
//   float angle = (float)(adc - minADC) / (maxADC - minADC);
//   float servoAngle = servoMinDeg + angle * (servoMaxDeg - servoMinDeg);

  // Serial.print("ADC: "); 
  
//   Serial.print("   Angle: "); Serial.println(servoAngle, 1);

//   // Optional: command servo and see result
//   // myServo.write((int)servoAngle);

  // delay(500);
}



