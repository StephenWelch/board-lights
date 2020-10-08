#include <Arduino.h>
#include <Wire.h>
#include <FastLED.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "plotter.h"
#include "util.h"
#include "filter.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE) && !defined(PARTICLE)
#include "Wire.h"
#endif

#define NUM_STRIP_LEDS 15
#define LEFT_STRIP_PIN 4
#define RIGHT_STRIP_PIN 5

#define IMU_INTERRUPT_PIN 2

#define GO_COLOR CRGB(0, 255, 0)
#define STOP_COLOR CRGB(255, 0, 0)

#define OUTPUT_READABLE_REALACCEL

CRGB leds[NUM_STRIP_LEDS];

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion quat;
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

bool errorState = false;
Plotter *plot;
ExponentialSmoothingFilter *filter;

void readImuBlocking() {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    while(fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    mpu.dmpGetQuaternion(&quat, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &quat);
    // mpu.dmpGetYawPitchRoll(ypr, &quat, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
}

void strobe(CRGB *leds, int num, const CRGB &color) {
    for(int i = 0; i < num; i++) {
        leds[i] = color * sin(i / num + millis());
    }
}

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    while (!Serial);

    // initialize device
    // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    // pinMode(IMU_INTERRUPT_PIN, INPUT);

    // verify connection
    // Serial.println(F("Testing device connections..."));
    // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    // Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-335);
    mpu.setYGyroOffset(-65);
    mpu.setZGyroOffset(-77);
    mpu.setZAccelOffset(1166); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(2);
        mpu.CalibrateGyro(2);
        // mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        // Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        // Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    FastLED.addLeds<NEOPIXEL, RIGHT_STRIP_PIN>(leds, NUM_STRIP_LEDS);

    plot = new Plotter();
    filter = new ExponentialSmoothingFilter(0.1);
}

void loop()
{
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    errorState = true;
    float brightness = 0;
    if(errorState) {
        fill_solid(&(leds[0]), NUM_STRIP_LEDS, CRGB(0, 0, 255));
    } else {
        // float adjusted_sin = sin(millis() / 1000.0 * 2.0);
        // float adjusted_cos = cos(millis() / 1000.0 * 2.0);
        // fill_solid(&(leds[0]), NUM_STRIP_LEDS, CRGB(interpolate(adjusted_sin, -1.0, 1.0, 0.0, 255.0), interpolate(adjusted_cos, -1.0, 1.0, 0.0, 255.0), 0));

        readImuBlocking();
        filter->filter(aaReal.x);

        // Deadzone?
        if(aaReal.x > 0) {
            brightness = smooth_interpolate_increase(filter->getFilteredValue(), 0, 8000.0, 0, 255);        
            fill_solid(&(leds[0]), NUM_STRIP_LEDS, CHSV(96, 255, brightness));
        } else {
            brightness = smooth_interpolate_decrease(filter->getFilteredValue(), -8000.0, 0, 0, 255);        
            fill_solid(&(leds[0]), NUM_STRIP_LEDS, CHSV(0, 255, brightness));
        }
    }
    
    FastLED.show();
    delay(10);

    plot->add("accel_x", aaReal.x);
    // plot->add("accel_y", aaReal.y);
    // plot->add("accel_z", aaReal.z);
    plot->add("brightness", brightness);
    plot->add("accel_x_filtered", filter->getFilteredValue());
    // plot->add("yaw", ypr[0] * 180/M_PI);
    // plot->add("pitch", ypr[1] * 180/M_PI);
    // plot->add("roll", ypr[2] * 180/M_PI);
    plot->commit();
}