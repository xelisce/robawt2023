#include <Arduino.h>
#include <Servo.h>
#include <TCA9548A.h>
#include <VL53L0X.h>
#include <VL53L1X.h>

int f_reading, fb_reading;
bool servos_change = true;

//* assign ports *//
#define FB_LIDAR 4 //front_bottom lidar channel
#define F_LIDAR 3 //front top lidar
#define CLAW_L 1 //S1
#define CLAW_R 2 //S2
#define CLAW_ARM 3 //S3
//* choose either ball or cube *//
#define pick_ball 1
#define pick_cube 0
//* choose if print *//
#define debug_print 1 
//* choose initial angles of servos *//
int claw_arm_angle = 180;
int claw_l_angle = 0;
int claw_r_angle = 130;
//* change trigger if needed *//
bool item_present() {
    //+45 for the diff sensors' offset physically
    return (f_reading +45 - fb_reading > 30 && fb_reading < 80);
}
//* change claw functions if needed *//
void claw_open() {
    claw_r_angle = 130; 
    claw_l_angle = 0;
    servos_change = true;
}
void claw_close_ball() {
    claw_r_angle = 25;
    claw_l_angle = 105;
    servos_change = true;
}
void claw_close_cube() { //cube
    claw_r_angle = 10;
    claw_l_angle = 120;
    servos_change = true;
}
void claw_up() {
    claw_arm_angle = 0;
    servos_change = true;
}
void claw_down() {
    claw_arm_angle = 180;
    servos_change = true;
}
void claw_service_up() {
    claw_arm_angle = 140;
    servos_change = true;
}


#define TCAADR 0x70
#define L0XADR 0x29
#define L1XADR 0x29
#define TCSADR 0x29

#define LEDPIN 3
#define SDAPIN 4
#define SCLPIN 5
#define SDA1PIN 6
#define SCL1PIN 7
#define TX0PIN 16
#define RX0PIN 17
#define PICOLEDPIN 25
#define SWTPIN 28

TCA9548A TopMux(SDAPIN, SCLPIN, &Wire, 400000);
VL53L0X f_lidar;
VL53L1X fb_lidar;
Servo claw_arm_servo, claw_l_servo, claw_r_servo;

int pickupState = 0;
long pickupStateTimer;

void setup() 
{
    Serial.begin(9600);
    #if debug_print
    while (!Serial) delay(10);
    Serial.println("Serial initialised");
    #endif

    pinMode(SWTPIN, INPUT);

    TopMux.begin();

    TopMux.select(F_LIDAR);
    f_lidar.setTimeout(500);
    while (!f_lidar.init()) Serial.println("FRONT LIDAR FAILED TO INIT");
    f_lidar.startContinuous();

    TopMux.select(FB_LIDAR);
    fb_lidar.setTimeout(500);
    while (!fb_lidar.init()) Serial.println("FRONT BOTTOM LIDAR FAILED TO INIT");
    fb_lidar.setDistanceMode(VL53L1X::Medium);
    fb_lidar.startContinuous(20);

    claw_arm_servo.attach(CLAW_ARM, 500, 2500);
    claw_l_servo.attach(CLAW_L, 500, 2500);
    claw_r_servo.attach(CLAW_R, 500, 2500);
    claw_arm_servo.writeMicroseconds(pwmangle(claw_arm_angle, 300));
    claw_l_servo.writeMicroseconds(pwmangle(claw_l_angle, 180));
    claw_r_servo.writeMicroseconds(pwmangle(claw_r_angle, 180));
}

void loop() 
{
    if (servos_change) {
        claw_arm_servo.writeMicroseconds(pwmangle(claw_arm_angle, 300));
        claw_l_servo.writeMicroseconds(pwmangle(claw_l_angle, 180));
        claw_r_servo.writeMicroseconds(pwmangle(claw_r_angle, 180));
        servos_change = false;
    }

    TopMux.select(F_LIDAR);
    if(f_lidar.available()) {
        f_reading = f_lidar.readRangeMillimeters();
    }
    TopMux.select(FB_LIDAR);
    if(fb_lidar.dataReady()) {
        fb_reading = fb_lidar.read(false);
    }

    if (digitalRead(SWTPIN)) {
        claw_service_up();     
    } else {
        claw_down();

        #if debug_print
        Serial.print("See item: ");
        Serial.println(item_present());
        #endif

        if (item_present() || pickupState != 0) {

            switch (pickupState)
            {
                case 0:
                    pickupStateTimer = millis();
                    pickupState ++;
                    break;

                case 1:
                    #if pick_cube
                    claw_close_cube();
                    #endif
                    #if pick_ball
                    // claw_down();
                    claw_close_ball();
                    #endif
                    if (millis() - pickupStateTimer > 1000) {
                        pickupStateTimer = millis();
                        pickupState ++; }
                    break;

                case 2:
                    // claw_close();
                    claw_up();
                    if (millis() - pickupStateTimer > 1000) {
                        pickupStateTimer = millis();
                        pickupState ++; }
                    break;

                case 3: 
                    claw_open();
                    // claw_up();
                    if (millis() - pickupStateTimer > 1000) {
                        pickupStateTimer = millis();
                        pickupState ++; }
                    break;

                case 4:
                    claw_down();
                    // claw_open();
                    if (millis() - pickupStateTimer > 1000) {
                        pickupStateTimer = millis();
                        pickupState = 0; }
                    break;
            }

            #if debug_print
            Serial.print("Pickup state: ");
            Serial.println(pickupState);
            #endif

        } else {
            claw_open();
            claw_down();
        }
    }
}

int pwmangle(double angle, double max_angle) //Servo PWM
{
    return (int)(angle/max_angle * 2000 + 500);
}