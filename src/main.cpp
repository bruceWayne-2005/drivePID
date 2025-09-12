#include <Arduino.h>
#include <PID_v1.h>


uint8_t encaPins[6];
uint8_t encbPins[6];

double pos[6], prevT[6];
double Input[6], Output[6], Setpoint[6];
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

PID motor0(&Input[0], &Output[0], &Setpoint[0], Kp, Ki, Kd, DIRECT);
PID motor1(&Input[1], &Output[1], &Setpoint[1], Kp, Ki, Kd, DIRECT);
PID motor2(&Input[2], &Output[2], &Setpoint[2], Kp, Ki, Kd, DIRECT);
PID motor3(&Input[3], &Output[3], &Setpoint[3], Kp, Ki, Kd, DIRECT);
PID motor4(&Input[4], &Output[4], &Setpoint[4], Kp, Ki, Kd, DIRECT);
PID motor5(&Input[5], &Output[5], &Setpoint[5], Kp, Ki, Kd, DIRECT);


void readEncoder(int i) {
    int b = digitalRead(encbPins[i]);
    int increment = (b > 0) ? 1 : -1;
    pos[i] += increment;

    unsigned long currT = micros();
    float deltaT = (currT - prevT[i]) / 1.0e6; // sec
    if (deltaT > 0){
        Input[i] = increment / deltaT;  // ticks/sec
    }
    prevT[i] = currT;
}

void readEncoder0() { readEncoder(0); }
void readEncoder1() { readEncoder(1); }
void readEncoder2() { readEncoder(2); }
void readEncoder3() { readEncoder(3); }
void readEncoder4() { readEncoder(4); }
void readEncoder5() { readEncoder(5); }

void setup() {
    attachInterrupt(digitalPinToInterrupt(encaPins[0]), readEncoder0, RISING);
    attachInterrupt(digitalPinToInterrupt(encaPins[1]), readEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(encaPins[2]), readEncoder2, RISING);
    attachInterrupt(digitalPinToInterrupt(encaPins[3]), readEncoder3, RISING);
    attachInterrupt(digitalPinToInterrupt(encaPins[4]), readEncoder4, RISING);
    attachInterrupt(digitalPinToInterrupt(encaPins[5]), readEncoder5, RISING);

    // PID Instances Setup
    motor0.SetMode(AUTOMATIC);
    motor1.SetMode(AUTOMATIC);
    motor2.SetMode(AUTOMATIC);
    motor3.SetMode(AUTOMATIC);
    motor4.SetMode(AUTOMATIC);
    motor5.SetMode(AUTOMATIC);
    
    // Output
    motor0.SetOutputLimits(-255, 255);
    motor1.SetOutputLimits(-255, 255);
    motor2.SetOutputLimits(-255, 255);
    motor3.SetOutputLimits(-255, 255);
    motor4.SetOutputLimits(-255, 255);
    motor5.SetOutputLimits(-255, 255);
}

void loop() {
    motor0.Compute();
    motor1.Compute();
    motor2.Compute();
    motor3.Compute();
    motor4.Compute();
    motor5.Compute();
}