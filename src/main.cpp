#include <Arduino.h>
#include <PID_v1.h>

/*
uint8_t encaPins[6]={PC9, PB8, PB9, PA5, PA6, PA7};
uint8_t encbPins[6]={PC8, PC6, PC5, PA12, PA11, PB12};

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
    Serial.println("debigging");
    pos[i] += increment;

    unsigned long currT = micros();
    float deltaT = (currT - prevT[i]) / 1.0e6; // sec
    if (deltaT > 0){
        Input[i] = increment / deltaT;  // ticks/sec
    }
    prevT[i] = currT;
}

void readEncoder0(){readEncoder(0);}
void readEncoder1(){readEncoder(1);}
void readEncoder2(){readEncoder(2);}
void readEncoder3(){readEncoder(3);}
void readEncoder4(){readEncoder(4);}
void readEncoder5(){readEncoder(5);}

void setup() {
    Serial.begin(115200);

    for (int i = 0; i<6; i++){
        pinMode(encaPins[i], INPUT);
        pinMode(encbPins[i], INPUT);

        pos[i] = 0;
        prevT[i] = micros();
        Input[i] = 0;
        Output[i] = 0;
        Setpoint[i] = 0;
    }
    

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
    Serial.println(pos[0]);

    motor0.Compute();
    motor1.Compute();
    motor2.Compute();
    motor3.Compute();
    motor4.Compute();
    motor5.Compute();

    delay(1000);
}
*/

// Motor Ctrl pin definitions
uint32_t dirPins[6] = {PC10, PC12, PC15, PH0, PC2, PC3};
uint32_t pwmPins[6] = {PC11, PD2, PA0, PA1, PC1, PC0};

void setMotor(int i, int pwm){
    if(pwm >= 0){
        digitalWrite(dirPins[i], LOW);
        analogWrite(pwmPins[i], pwm);
    }
    else{
        digitalWrite(dirPins[i], HIGH);
        analogWrite(pwmPins[i], pwm);
    }
}



// Encoder Pin definitions
uint32_t encaPins[6]={PC9, PB8, PA5, PA6, PA7, PB10};
uint32_t encbPins[6]={PC8, PC6, PA12, PA11, PB12, PB15};

// Encoder Data
volatile long tickCount[6] = {0};
volatile long pos[6] = {0};
double tps[6] = {0};
unsigned long prevT = 0;

// Encoder Read Functions
void readEncoder(int i) {
    int b = digitalRead(encbPins[i]);
    int increment = (b > 0) ? 1 : -1;
    pos[i] += increment;
    tickCount[i] += increment;
}
void readEncoder0(){readEncoder(0);}
void readEncoder1(){readEncoder(1);}
void readEncoder2(){readEncoder(2);}
void readEncoder3(){readEncoder(3);}
void readEncoder4(){readEncoder(4);}
void readEncoder5(){readEncoder(5);}



// PID
double pwm[6], setTps[6];
double Kp = 0.8, Ki = 0.1, Kd = 0.4;

PID motor0(&tps[0], &pwm[0], &setTps[0], Kp, Ki, Kd, DIRECT);
PID motor1(&tps[1], &pwm[1], &setTps[1], Kp, Ki, Kd, DIRECT);
PID motor2(&tps[2], &pwm[2], &setTps[2], Kp, Ki, Kd, DIRECT);
PID motor3(&tps[3], &pwm[3], &setTps[3], Kp, Ki, Kd, DIRECT);
PID motor4(&tps[4], &pwm[4], &setTps[4], Kp, Ki, Kd, DIRECT);
PID motor5(&tps[5], &pwm[5], &setTps[5], Kp, Ki, Kd, DIRECT);



void setup(){
    Serial.begin(115200);

    
    // PinMode Setup
    for (int i = 0; i < 6; i++){
        pinMode(dirPins[i], OUTPUT);
        pinMode(pwmPins[i], OUTPUT);
        pinMode(encaPins[i], INPUT);
        pinMode(encbPins[i], INPUT);

        setTps[i] = 500;
    }
    // Encoder Interrupts
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

void loop(){
    // Update tps of every motor
    unsigned long currT = millis();
    if (currT - prevT >= 10){ // every 10 ms
        noInterrupts();
        long ticks[6];
        for (int i = 0; i < 6; i++){
            ticks[i] = tickCount[i];
            tickCount[i] = 0; 
        }
        interrupts();

        for (int i = 0; i < 6; i++){
            tps[i] = ticks[i] / ((currT - prevT) / 1000.0); // ticks/sec
        }
        prevT = currT;
    

        motor0.Compute();
        motor1.Compute();
        motor2.Compute();
        motor3.Compute();
        motor4.Compute();
        motor5.Compute();


        for (int i = 0; i < 6; i++){
            Serial.print("Motor");
            Serial.print(i);
            Serial.print(" PWM: ");
            Serial.print(pwm[i]);
            Serial.print(" TPS: ");
            Serial.println(tps[i]);
        }
        Serial.println();


        for (int i = 0; i < 6; i++){
            //if (pwm[i] > 50 || pwm[i] < -50){
                setMotor(i, pwm[i]);
            //}
            //else{
            //    setMotor(i, 0);
            //}
        }
        
        Serial.println();
    }
}