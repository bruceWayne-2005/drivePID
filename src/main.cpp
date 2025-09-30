#include <Arduino.h>
#include <PID_v1.h>


// Constants
#define pi 3.14
#define ENCPR 1320 // not correct
#define WHEEL_RADIUS 0.12


// Motor Ctrl definitions
double target_linear_velocity = 0.5;

uint32_t dirPins[6] = {PC4, PC12, PD2, PC11, PC2, PC10};
uint32_t pwmPins[6] = {PA8, PB0, PA0, PA1, PB1, PB6};

void setMotor(int i, int pwm){
    if(pwm >= 0){
        digitalWrite(dirPins[i], LOW);
        analogWrite(pwmPins[i], pwm);
    }
    else{
        digitalWrite(dirPins[i], HIGH);
        analogWrite(pwmPins[i], -pwm);
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
double pwm[6] = {0};
double setTps[6];
double Kp = 0.05, Ki = 0.2, Kd = 0.0;

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
        pinMode(encaPins[i], INPUT_PULLUP);
        pinMode(encbPins[i], INPUT_PULLUP);
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

        
        double wl = 0; // Angular Velocity of Left
        double wr = 0; // Angular Velocity of Right
        
        double dt = (currT - prevT) / 1000.0;
        prevT = currT;
        for (int i = 0; i < 6; i++){
            tps[i] = ticks[i] / dt; // ticks/sec
            // w = 2*pi*tps/encpr
            if (i < 3){wl += 2*pi*tps[i]/ENCPR;}
            else{wr += 2*pi*tps[i]/ENCPR;}
        }
        wl = wl/3;
        wr = wr/3;

        // read data from imu

        // compute pid input->imu data vel set->target_angular_velocity
        // setlwl-output and setwr+output

        double setWl = target_linear_velocity/WHEEL_RADIUS;
        double setWr = -target_linear_velocity/WHEEL_RADIUS;
        for (int i = 0; i < 6; i++){
            if (i < 3){setTps[i] = setWl*ENCPR/(2*pi);}
            else{setTps[i] = setWr*ENCPR/(2*pi);}
        }
        

        // Computes the output for motors
        motor0.Compute();
        motor1.Compute();
        motor2.Compute();
        motor3.Compute();
        motor4.Compute();
        motor5.Compute();
        for(int i = 0; i < 6; i++){
            setMotor(i, pwm[i]);
        }

        double linear_vel = WHEEL_RADIUS*(wl + wr)/2;
        for (int i = 0; i < 6; i++){
            Serial.print("Motor");
            Serial.print(i);
            Serial.print(" PWM: ");
            Serial.print(pwm[i]);
            Serial.print(" TPS: ");
            Serial.println(tps[i]);
        }
        Serial.println();

        Serial.print("Linear Velocity: ");
        Serial.println(linear_vel);        
    }

    /*for (int i = -255; i < 256; i++){
        setMotor(5, i);
        delay(10);
    }
    for (int i = 255; i >= -255; i--){
        setMotor(5, i);
        delay(10);
    }*/
}