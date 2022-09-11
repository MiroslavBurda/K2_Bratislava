#include <Arduino.h>
#include <format.h>
using fmt::print;
#include "RBControl_manager.hpp"
#include <Wire.h>
#include <Servo.h>
#include "time.hpp"
#include <atomic>

rb::Manager* man = nullptr;

Servo servo0; 
int position_servo0 = 40;
int power_motor = 70;

static const uint32_t i2c_freq = 400000;
static const uint8_t L_G_pin = rb::EB5;

byte L_G_light = 0; 




std::atomic_int left_enc; 

bool casovac1() {        // musí být bool, musí vracet true, může jich být 10 
    left_enc = left_enc+1;
    return true;
}

void setup() {
    rb::Manager m(false);  
    auto& batt = m.battery();
    batt.setCoef(8.65);
    printf("\n\nBATTERY CALIBRATION INFO: %d (raw) * %.2f (coef) = %.2f V\n\n\n", batt.raw(), batt.coef(), batt.raw()*batt.coef()/1000 );
    man = &m;
    m.expander().pinMode(rb::LED_BLUE, OUTPUT);
    m.expander().pinMode(rb::LED_GREEN, OUTPUT);
    m.expander().pinMode(rb::LED_YELLOW, OUTPUT);
    m.expander().pinMode(rb::LED_RED, OUTPUT);
    m.expander().pinMode(rb::SW1, INPUT_PULLUP);
    m.expander().pinMode(rb::SW2, INPUT_PULLUP);
    m.expander().pinMode(rb::SW3, INPUT_PULLUP);
    Serial.begin (115200);
    Serial.print ("Starting.../n");
    pinMode(L_G_pin, OUTPUT); 
    servo0.attach(32);
    servo0.write(position_servo0);
    man->schedule(500, casovac1); // je volaný každých 500 ms 
}

#define LEFT_MOTOR  rb::MotorId::M1
#define RIGHT_MOTOR  rb::MotorId::M2

 timeout send_data { msec(100) }; // timeout zajistuje posilani dat do PC kazdych 500 ms

bool sw1() { return !man->expander().digitalRead(rb::SW1); }

void loop() // this part works in cycle 
{
    man->leds().yellow(sw1());
    if (send_data) {
        send_data.ack();
        // Serial.println (millis());
        if (L_G_light == 0) L_G_light = 1; else  L_G_light = 0;
        //digitalWrite(L_G_pin, L_G_light); 
        man->leds().green(L_G_light);
        Serial.println(left_enc);
        // digitalWrite(LED_GREEN, L_G_light);
        
    }
    if(Serial.available()) {
        char c = Serial.read();
        switch(c) {
        case 't':
                if (position_servo0 >= 5)  position_servo0 = position_servo0 -5;               
                servo0.write(position_servo0);
                Serial.write(" 0: "); 
                Serial.print(position_servo0);
                break;
            case 'u':
                if (position_servo0 <= 175)  position_servo0 = position_servo0 +5;               
                servo0.write(position_servo0);
                Serial.write(" 0: "); 
                Serial.print(position_servo0);
                break;
            case 'w':
                man->setMotors().power(LEFT_MOTOR, power_motor)
                                .power(RIGHT_MOTOR, power_motor)
                                .set();
                break;
            case 's':
                man->setMotors().power(LEFT_MOTOR, -power_motor)
                                .power(RIGHT_MOTOR, -power_motor)
                                .set();
                break;
            case 'a':
                man->setMotors().power(LEFT_MOTOR, -power_motor)
                                .power(RIGHT_MOTOR, power_motor)
                                .set();
                break;
            case 'd':
                man->setMotors().power(LEFT_MOTOR, power_motor)
                                .power(RIGHT_MOTOR, -power_motor)
                                .set();
                break;
            case '1' ... '9':
                man->motor(LEFT_MOTOR)->drive(256 * (c - '0'), 64, nullptr);
                man->motor(RIGHT_MOTOR)->drive(256 * (c - '0'), 64, nullptr); // 96 tik; na otacku 
                break;
            case ' ':
                man->setMotors().power(LEFT_MOTOR, 0)
                                .power(RIGHT_MOTOR, 0)
                                .set();
                break;
            default:
                Serial.write(c);
                break;
        } 
    }
}

