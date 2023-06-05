#include "I2Cdev.h"
#include <SoftwareSerial.h>

// GPS Imports.
#include <TinyGPSPlus.h>
// MPU Imports.
#include <MPU6050_6Axis_MotionApps_V6_12.h>
// UltraSonic Imports.
 #include <NewPing.h>

struct Waypoint
{
    double lat;
    double lon;
};

class MotorController
{
    class Motor
    {
        const static int GO_PHASE = 0;
        const static int RAMP_PHASE = 1;
        const static int RAMP_WAIT_PHASE = 2;
        const static int STOP_PHASE = 3;

        const static int STEP_MILLIS = 50; // ms inbetween go calls.

    private:
        int dirPin;    // Direction Pin.
        int pwmPin;    // Speed Pin.
        int direction; // Direction State.
        double speed;  // Speed State.

        // Control Variables for the async acceleration.
        int phase;       // Controls what work gets done on each update call.
        long last_time;  // Time of the last ramp phase.
        long start_time; // Time of the newest acceleration instruction.

        // Acceleration Settings: Variables
        long duration;    // How long to accelerate for, from current_speed to new_speed;
        double new_speed; // The new speed to approach.
        double rate;      // The rate of change to add to speed each phase.

        /** Private Overload asyncRampSpeed, this gets called each update when phase == RAMP_PHASE
         * @param time: the current time of the motor controller.
         * Sets the time, increments speed by the current rate, which is in relation with the duration and new speed.
         */
        void asyncRampSpeed(long time)
        {
            // Increment variables.
            this->last_time = time;
            this->speed += rate;
            if (this->last_time - this->start_time >= duration)
            { // If the duration has been hit, end the ramp speed event.
                this->speed = new_speed;
                this->last_time = 0;
                this->rate = 0;
                this->start_time = 0;
                this->phase = GO_PHASE;
            }
            else
            {
                this->phase = RAMP_WAIT_PHASE; // If icremented and not done, wait STEP_MILLIS.
            }
        }

    public:
        Motor(int dirPin, int pwmPin)
        {
            this->dirPin = dirPin;
            this->pwmPin = pwmPin;

            this->direction = LOW;
            this->speed = 0;

            this->new_speed = 0;
            this->rate = 0;

            this->start_time = 0;
            this->last_time = 0;

            this->phase = STOP_PHASE;

            pinMode(dirPin, OUTPUT);
            pinMode(pwmPin, OUTPUT);
        }
        void setDirection(uint8_t direction)
        {
            this->direction = direction;
        }
        void setSpeed(int speed)
        {
            this->speed = speed;
            this->new_speed = speed;
        }
        /** Public Overloaded asyncRampSpeed
        * @param newSpeed: The new speed to approach.
        * @param duration: The amount of time in ms to approach the new speed.
          Gets called once to set the settings of the motor, each update call will slowly adjuct the current settings to the new ones.
        */
        void asyncRampSpeed(int newSpeed, int duration)
        {
            this->new_speed = newSpeed;
            this->start_time = millis();
            this->duration = duration;
            this->rate = ((this->new_speed - this->speed) / (this->duration / STEP_MILLIS));
            // Serial.print((this->new_speed - this->speed));
            // Serial.print(" / ");
            // Serial.print((this->duration / STEP_MILLIS));
            // Serial.print(" = ");
            // Serial.print(this->rate);
            // Serial.println();
            this->phase = RAMP_PHASE;
        }
        /** update
         * @param time: the current time of the Motor Controller.
         * Calls the corresponding work for the function.
         * Adjusts speed, and calls go unless phase == STOP_PHASE.
         */
        void update(long time)
        {
            if (phase == STOP_PHASE)
            {
                return;
            }
            if (phase == RAMP_PHASE)
            {
                asyncRampSpeed(time);
            }
            else if (phase == RAMP_WAIT_PHASE)
            {
                if (time - last_time > STEP_MILLIS)
                {
                    phase = RAMP_PHASE;
                }
            }
            go();
        }
        /** go
          Applies current state as voltage to the motors.
        */
        void go()
        {
            digitalWrite(dirPin, direction);
            analogWrite(pwmPin, speed);
        }
        void debug()
        {
            Serial.print(speed);
            Serial.print(", ");
            Serial.print(rate);
            Serial.print(", ");
            Serial.print(last_time);
        }
    };

private:
    Motor *motorRight;
    Motor *motorLeft;
    int accellDuration;

    void update(long time)
    {
        this->motorLeft->update(time);
        this->motorRight->update(time);

        // this->motorLeft->debug();
        // Serial.print("  |  ");
        // this->motorRight->debug();
        // Serial.println();
    }

public:
    MotorController(int dirRPin, int pwmRPin, int dirLPin, int pwmLPin)
    {
        this->motorRight = new Motor(dirRPin, pwmRPin);
        this->motorLeft = new Motor(dirLPin, pwmLPin);
        this->accellDuration = 1000;
    }
    void go()
    {
        update(millis());
        this->motorLeft->go();
        this->motorRight->go();
    }
    void handle_joystick()
    {
        int x = analogRead(A0);
        int y = analogRead(A1);
        if (y > 512)
        {
            this->motorLeft->setDirection(LOW);
            this->motorLeft->asyncRampSpeed(map(y, 512, 1024, 0, 255), accellDuration);
        }
        if (y < 512)
        {
            this->motorLeft->setDirection(HIGH);
            this->motorLeft->asyncRampSpeed(map(y, 512, 0, 0, 255), accellDuration);
        }
        if (x > 512)
        {
            this->motorRight->setDirection(LOW);
            this->motorRight->asyncRampSpeed(map(x, 512, 1024, 0, 255), accellDuration);
        }
        if (x < 512)
        {
            this->motorRight->setDirection(HIGH);
            this->motorRight->asyncRampSpeed(map(x, 512, 0, 0, 255), accellDuration);
        }
    }

    void autoCar(long currlon, long currlat, int leftdis, int centdis, int rightdis, int[] waypointlon, int[] waypointlat, int waypoint_num, int yaw, int courseToWaypoint)
    {   
      if (((unsigned long)TinyGPSPlus::distanceBetween( gps.location.lat(),gps.location.lng(),waypointlon[waypoint_num],waypointlat[waypoint_num])/1000) == 5)
        { // the long and lat are in area of the current waypoint
            waypoint_num++;
        }
        else
        { // we are not
            // check to see if there is a sensor in the way else move forward Move forward
            if (centdis > 100)
            {
                // move forward
                this->motorLeft->setDirection(LOW);
                this->motorRight->setDirection(LOW);
                this->motorLeft->asyncRampSpeed(50, accellDuration);
                this->motorRight->asyncRampSpeed(50, accellDuration);
                if (yaw < courseToWaypoint - 4) // turn right
                {
                    this->motorLeft->setDirection(LOW);
                    this->motorLeft->asyncRampSpeed(50, accellDuration);
                }
                else if (yaw > courseToWaypoint + 4) // turn to the left
                {
                    this->motorRight->setDirection(LOW);
                    this->motorRight->asyncRampSpeed(50, accellDuration);
                }
            }
            else
            {
                // check left and right
                if (leftdis > 100) // left is clear
                {
                    this->motorRight->setDirection(LOW);
                    this->motorRight->asyncRampSpeed(50, accellDuration);
                }
                else if (rightdis > 100) // if left is not clear and right is clear
                {
                    this->motorLeft->setDirection(LOW);
                    this->motorLeft->asyncRampSpeed(50, accellDuration);
                }
                else // completely covered
                {
                    // move backward
                    this->motorLeft->setDirection(HIGH);
                    this->motorRight->setDirection(HIGH);
                    this->motorLeft->asyncRampSpeed(50, accellDuration);
                    this->motorRight->asyncRampSpeed(50, accellDuration);
                }
            }
            // check the direction then change to be facing the direction of the next waypoint
        }
    }
    void split(uint8_t dir1, uint8_t dir2, int speed1, int speed2)
    {
        this->motorLeft->setDirection(dir1);
        this->motorRight->setDirection(dir2);
        this->motorLeft->asyncRampSpeed(speed1, accellDuration);
        this->motorRight->asyncRampSpeed(speed2, accellDuration);
    }
    void forward()
    {
        // test Forward
        //  setMotor(LOW, LOW, 50,  50);
        this->motorLeft->setDirection(LOW);
        this->motorRight->setDirection(LOW);
        this->motorLeft->asyncRampSpeed(255, accellDuration);
        this->motorRight->asyncRampSpeed(255, accellDuration);
    }
    void forward(int speed)
    {
        // test Forward
        //  setMotor(LOW, LOW, 50,  50);
        this->motorLeft->setDirection(LOW);
        this->motorRight->setDirection(LOW);
        this->motorLeft->asyncRampSpeed(speed, accellDuration);
        this->motorRight->asyncRampSpeed(speed, accellDuration);
    }
    void backward()
    {
        // Test Backward
        //  setMotor(HIGH, HIGH, 50, 50);
        this->motorLeft->setDirection(HIGH);
        this->motorRight->setDirection(HIGH);
        this->motorLeft->asyncRampSpeed(255, accellDuration);
        this->motorRight->asyncRampSpeed(255, accellDuration);
    }
    void backward(int speed)
    {
        // Test Backward
        //  setMotor(HIGH, HIGH, 50, 50);
        this->motorLeft->setDirection(HIGH);
        this->motorRight->setDirection(HIGH);
        this->motorLeft->asyncRampSpeed(speed, accellDuration);
        this->motorRight->asyncRampSpeed(speed, accellDuration);
    }
    void spinL()
    {
        // Test spinL
        //  setMotor(HIGH, LOW, 50, 50);
        this->motorLeft->setDirection(HIGH);
        this->motorRight->setDirection(LOW);
        this->motorLeft->asyncRampSpeed(255, accellDuration);
        this->motorRight->asyncRampSpeed(255, accellDuration);
    }
    void spinL(int speed)
    {
        // Test spinL
        //  setMotor(HIGH, LOW, 50, 50);
        this->motorLeft->setDirection(HIGH);
        this->motorRight->setDirection(LOW);
        this->motorLeft->asyncRampSpeed(speed, accellDuration);
        this->motorRight->asyncRampSpeed(speed, accellDuration);
    }
    void spinR()
    {
        // Test spinR
        //  setMotor(LOW, HIGH, 50,50);
        this->motorLeft->setDirection(LOW);
        this->motorRight->setDirection(HIGH);
        this->motorLeft->asyncRampSpeed(255, accellDuration);
        this->motorRight->asyncRampSpeed(255, accellDuration);
    }
    void spinR(int speed)
    {
        // Test spinR
        //  setMotor(LOW, HIGH, 50,50);
        this->motorLeft->setDirection(LOW);
        this->motorRight->setDirection(HIGH);
        this->motorLeft->asyncRampSpeed(speed, accellDuration);
        this->motorRight->asyncRampSpeed(speed, accellDuration);
    }
    void skidL()
    {
        // Test skidL
        //  setMotor(HIGH, LOW, 50, 0);
        this->motorLeft->setDirection(HIGH);
        this->motorRight->setDirection(LOW);
        this->motorLeft->asyncRampSpeed(255, accellDuration);
        this->motorRight->asyncRampSpeed(0, accellDuration);
    }
    void skidL(int speed)
    {
        // Test skidL
        //  setMotor(HIGH, LOW, 50, 0);
        this->motorLeft->setDirection(HIGH);
        this->motorRight->setDirection(LOW);
        this->motorLeft->asyncRampSpeed(speed, accellDuration);
        this->motorRight->asyncRampSpeed(0, accellDuration);
    }
    void skidR()
    {
        // Test skidR
        //  setMotor(LOW, HIGH, 0, 50);
        this->motorLeft->setDirection(LOW);
        this->motorRight->setDirection(HIGH);
        this->motorLeft->asyncRampSpeed(0, accellDuration);
        this->motorRight->asyncRampSpeed(255, accellDuration);
    }
    void skidR(int speed)
    {
        // Test skidR
        //  setMotor(LOW, HIGH, 0, 50);
        this->motorLeft->setDirection(LOW);
        this->motorRight->setDirection(HIGH);
        this->motorLeft->asyncRampSpeed(0, accellDuration);
        this->motorRight->asyncRampSpeed(speed, accellDuration);
    }
    void brake()
    {
        // Test brake
        //  setMotor(LOW, LOW, 0, 0);
        this->motorLeft->asyncRampSpeed(0, accellDuration);
        this->motorRight->asyncRampSpeed(0, accellDuration);
    }
    void setAccellDuration(int dur)
    {
        this->accellDuration = dur;
    }
};

class UltraSonic
{
public:
    static const uint8_t cache_size = 5;
    uint8_t cache[cache_size];
    int trig, echo;
    int red, green;
    uint8_t cm, zero_counter, threshold;

    bool inThreshold;

    NewPing *sensor;

    UltraSonic(int trig, int echo, int red, int green, long threshold)
    {
        this->trig = trig;
        this->echo = echo;
        this->red = red;
        this->green = green;

        this->threshold = threshold;
        this->inThreshold = false;

        pinMode(this->trig, OUTPUT);
        pinMode(this->echo, INPUT);
        pinMode(this->red, OUTPUT);
        pinMode(this->green, OUTPUT);

        sensor = new NewPing(trig, echo, 200);
    }

    void update()
    {
        cm = sensor->ping_cm();
        if (0 < cm && cm < threshold)
        {
            digitalWrite(red, HIGH);
            digitalWrite(green, LOW);
        }
        else
        {
            digitalWrite(red, LOW);
            digitalWrite(green, HIGH);
        }

        // Serial.print(cm);
        // Serial.print(",   ");
    }

    int getDist()
    {
        return sensor->ping_cm();
    }
};

#define RXPin 12
#define TXPin 13
#define GPSBaud 9600
#define INTERRUPT_PIN 19
#define GPS_BUTTON 18
#define MOTOR_SWITCH 4
#define STATE_SWITCH 5

// Waypoint waypoints[10];
int waypointlong[10];
int waypointlat[10];
int numWaypoints = 0;
int currWaypoint = 0;

MotorController mc(8, 9, 10, 11);
String instruction = "";
int temp;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

Waypoint NORTH_POLE = {91.000000, 135.000000};

unsigned long distanceKmToWaypoint;
double courseToWaypoint;

uint8_t devStatus;
uint8_t mpuIntStatus;
uint8_t fifoBuffer[64];

uint16_t packetSize;
uint16_t fifoCount;

float ypr[3]; // [yaw, pitch, roll]
float yaw;

volatile bool mpuInterrupt = false;
bool dmpReady = false;

void dmpDataReady()
{
    mpuInterrupt = true;
}

void gpsEvent()
{
    if (numWaypoints < 10)
    {
        float lat = gps.location.lat(), lon = gps.location.lng();
        if (lat != 0.0 && lon != 0.0)
        {
            waypoints[numWaypoints].lat = lat;
            waypoints[numWaypoints].lon = lon;
            waypointlat[numWaypoints] = lat;
            waypointlong[numWaypoints] = lon;
            numWaypoints++;
        }
        for (int i = 0; i < 10; i++)
        {
            // Serial.print(waypoints[i].lat);
            // Serial.print(":");
            // Serial.print(waypoints[i].lon);
            // Serial.print(", ");
          Serial.print(waypointlat[i]);
            Serial.print(":");
            Serial.print(waypointlong[i]);
            Serial.print(", ");
        }
        Serial.println();
    }
    else
    {
        Serial.println("WAYPOINTS FULL");
    }
}

MPU6050 mpu;

Quaternion q;        // [w, x, y, z]
VectorFloat gravity; // [x, y, z]

// UltraSonic Variables.
UltraSonic left_us = UltraSonic(6, 7, 36, 35, 50);
UltraSonic center_us = UltraSonic(24, 22, 34, 33, 50);
UltraSonic right_us = UltraSonic(28, 26, 32, 31, 50);

void setup()
{
    Wire.begin();
    Wire.setClock(400000);

    Serial.begin(115200);

    ss.begin(GPSBaud);

    while (!Serial)
        ;

    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0)
    {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    pinMode(31, OUTPUT);
    pinMode(33, OUTPUT);
    pinMode(35, OUTPUT);

    pinMode(INTERRUPT_PIN, INPUT);

    pinMode(GPS_BUTTON, INPUT_PULLUP);
    pinMode(27, OUTPUT);
    pinMode(29, OUTPUT);
    pinMode(MOTOR_SWITCH, INPUT);
    pinMode(STATE_SWITCH, INPUT);
    attachInterrupt(digitalPinToInterrupt(GPS_BUTTON), gpsEvent, FALLING);
}

void loop()
{

    if (!dmpReady)
    {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = fmod((ypr[0] * 180 / M_PI) + 360, 360);
        }
    }

    if (gps.location.isValid())
    {
        distanceKmToWaypoint =
            (unsigned long)TinyGPSPlus::distanceBetween(
                gps.location.lat(),
                gps.location.lng(),
                NORTH_POLE.lat,
                NORTH_POLE.lon) /
            1000;

        if (distanceKmToWaypoint < 5)
        {
            digitalWrite(27, HIGH);
            digitalWrite(29, LOW);
        }
        else if (distanceKmToWaypoint < 3)
        {
            digitalWrite(27, LOW);
            digitalWrite(29, HIGH);
        }
        else
        {
            digitalWrite(27, LOW);
            digitalWrite(29, LOW);
        }
        courseToWaypoint =
            fmod(TinyGPSPlus::courseTo(
                     gps.location.lat(),
                     gps.location.lng(),
                     NORTH_POLE.lat,
                     NORTH_POLE.lon),
                 360);

        if (yaw < courseToWaypoint - 4) // to the left
        {
            digitalWrite(31, HIGH);
            digitalWrite(33, LOW);
            digitalWrite(35, LOW);
        }
        else if (yaw > courseToWaypoint + 4) // to the right
        {
            digitalWrite(31, LOW);
            digitalWrite(33, LOW);
            digitalWrite(35, HIGH);
        }
        else // on course
        {
            digitalWrite(31, LOW);
            digitalWrite(33, HIGH);
            digitalWrite(35, LOW);
        }
    }
    left_us.update();
    center_us.update();
    right_us.update();

    Serial.print("State: ");
    Serial.print(digitalRead(STATE_SWITCH));

    Serial.print(", ");
    Serial.print("Motor: ");
    Serial.print(digitalRead(MOTOR_SWITCH));
    Serial.println();

    // mc.handle_joystick();
    if (digitalRead(STATE_SWITCH) == HIGH) // State_Switch is HIGH
    {   
      //double currlon, double currlat, int leftdis, int centdis, int rightdis, int[] waypointlon, int[] waypointlat, int waypoint_num, float yaw, double courseToWaypoint
        mc.autoCar(gps.location.lng(), gps.location.lng(), left_us.getDist(), center_us.getDist(), right_us.getDist(), waypointlong , waypointlat , currWaypoint , yaw , courseToWaypoint);
    }
    else // State_Switch is LOW
    {
        mc.handle_joystick();
    }
    mc.go();

    smartDelay(0);
}
static void smartDelay(unsigned long ms)
{
    unsigned long start = millis();
    do
    {
        while (ss.available())
            gps.encode(ss.read());
    } while (millis() - start < ms);
}