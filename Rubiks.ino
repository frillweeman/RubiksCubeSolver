#include <Encoder.h>
#include <Servo.h>
#include <PID_v1.h>

class Motor {
private:
  Servo servo;
  
public:
  void begin(int pin) {
    servo.attach(pin);
  }
  
  void set(int s) {
    int calculatedSpeed = map(s, -100, 100, 37, 143);
    servo.write(calculatedSpeed);
  }
};

double setpoint, input, output;

// TUNE HERE
double kP = 2.0,
       kI = 0.0,
       kD = 0.0;


Motor motorLeft;
Encoder myEnc(2, 3);
PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT);

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  motorLeft.begin(9);

  setpoint = 90.0;
  pid.SetMode(AUTOMATIC);

  digitalWrite(13, LOW);
  delay(3000);
  digitalWrite(13, HIGH);
}

void loop() {
//  static long oldPosition  = 0;
//  long newPosition = myEnc.read();
//  if (newPosition != oldPosition) {
//    oldPosition = newPosition;
//    Serial.println(newPosition);
//  }

  input = myEnc.read();
  pid.Compute();
  Serial.println(output);
  motorLeft.set(output);
}