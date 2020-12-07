// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
// I have to use HMComAssistant  and AT+RENEW command to fix the bug (RX254 or noise all the time), connection in the link  http://techienoise.com/upgrading-firmware-to-hm-10-cc2541-ble-4-0/
//Be careful with the accelerometer, still have noise in the beginning
//
//We must wait until the accel value is small (speed is reset to zero), we can only walk and record speed data correctly after the waiting period
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "math.h"
#include <SoftwareSerial.h>


#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <setPWMFrequency.h>
#include <HM10MasterSerialInit.h>
#include <InitMotor.h>


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
using namespace std;
MPU6050 mpu;
//MPU6050 mpu1(0x69); // <-- use for AD0 high

SoftwareSerial SWSerial(7, 8); // RX, TX
/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using SWSerial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

//#define OUTPUT_READABLE_ACCEL

//#define OUTPUT_READABLE_GRAVITY

//---------------------------------------------------------------------

//#define AccelSensitivity_2G 

//#define AccelSensitivity_4G

#define AccelSensitivity_8G


//---------------------------------------------------------------------
int duty, gradualStopDuty, new_duty, duty_set;     // how many points to fade the LED by
int startup_safe_duty=90, turnoff_threshold=30, safe_duty_threshold=110,non_stop_threshold=85;
int num_loop=0;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

bool stopbyOther,stopbymyself,update_duty=false;;

bool adapttomyself;
int TXAdaptedSignal=2, RXAdaptedSignal=2, PilotSignal=3;

// MPU control/status vars
bool dmpReady = false,fst_peak=true,MtorIsMoving=false,lost_connection=true;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
long time1=0,time_old,step_start_time,half_step_time,step_peak_time,sample_time;
unsigned long pilot_send_time,pilot_receive_time;
float delta_t,SumMagAccel, turnoff_Ratio=0.7;
//float delta_time;
int run1=1,j,peak_count;

//============================
//For Normal and Faster Speed
//============================
//float AccelMagThreshold=1.5;



//============================
//For Fastest Speed;
//============================
float AccelMagThreshold=1,RoCh,RoChThreshold=8;// Rate of Accel change
const int NumSamplesToSetZero=2;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  
float aaWorldX;
float aaWorldY;
float aaWorldZ;

float peak_speed,avg_peak_speed,ratio=1,peak_speeds[5],abs_x;// we will monitor 4 previous peak speed values
//float xx[5]={1,2,3,4,5};
//============================

int const NumOfSamples=1;// num of sample to average

int16_t AccelX[NumOfSamples+1], AccelY[NumOfSamples+1], AccelZ[NumOfSamples+1];

//============================

int count=0;            // initial value to count the number of samples to compute average

VectorFloat gravity;    // [x, y, z]            gravity vector, ///////////////////class VectorFloat is defined in helper_3dmath.h
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float vx,vy,vz;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

int RX_Data_BLE=2;


class AvgAccel{
  public:
  float x=0;
  float y=0;
  float z=0;
   float mag(){
    return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    }
  
  };
  
//int const NumOfAccelSampletoZero=3;

AvgAccel AVAWorld, AVAWorld1,AVAWorld_Zero;
float AVAWorldMagSeries[NumSamplesToSetZero];


//AvgAccel aaWorld5[NumOfAccelSampletoZero];

class Speed{
  public:
  float x;
  float y;
  float z;
  float mag(){
    return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
  }
  
  };
  
Speed spd[2],Spds[4]; // WE NEED TWO SPDs IN ORDER TO CHECK THE RESET CONDITION

//void AverageAccel(AvgAccel *AAccel) 
//{
//  
//              if (count==0)   // 1st average
//              {
//                do
//                {
//                  mpu.dmpGetQuaternion(&q, fifoBuffer);
//                  mpu.dmpGetAccel(&aa, fifoBuffer);
//                  mpu.dmpGetGravity(&gravity, &q);
//                  
//                  #ifdef AccelSensitivity_2G
//                  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//                  #endif
//
//                                    
//                  #ifdef AccelSensitivity_4G
//                  dmpGetLinearAccel_4G(&aaReal, &aa, &gravity);
//                  #endif
//
//                  #ifdef AccelSensitivity_8G
//                  dmpGetLinearAccel_8G(&aaReal, &aa, &gravity);
//                  #endif
//                  
//                  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//                  
//                  AccelX[count]=aaWorld.x;
//                  AccelY[count]=aaWorld.y;
//                  AccelZ[count]=aaWorld.z;
//                  count++;
//                  }
//                while(count<NumOfSamples);
//    
//                  int16_t aaWorldX_sum=0;
//                  int16_t aaWorldY_sum=0;
//                  int16_t aaWorldZ_sum=0;
//                
//                  for (int i=0;i<NumOfSamples;i++)
//                  {
//                    aaWorldX_sum+=AccelX[i];
//                    aaWorldY_sum+=AccelY[i];
//                    aaWorldZ_sum+=AccelZ[i];
//                  }
//                  ///=== For 2g sensitivity=====
//                  #ifdef AccelSensitivity_2G
//                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/8192;
//                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/8192;
//                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/8192;
//                  #endif
//                  
//                  ///=== For 4G sensitivity=====
//                  #ifdef AccelSensitivity_4G
//                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/4096;
//                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/4096;
//                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/4096;
//                  #endif
//
//                  ///=== For 8G sensitivity=====
//                  #ifdef AccelSensitivity_8G
//                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/2048;
//                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/2048;
//                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/2048;
//                  #endif
//                  
//                  
//                }
//
//            else
//            {
////              mpu.dmpGetQuaternion(&q, fifoBuffer);
////              mpu.dmpGetAccel(&aa, fifoBuffer);
////              mpu.dmpGetGravity(&gravity, &q);
////              
////                  #ifdef AccelSensitivity_2G
////                  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
////                  #endif
////
////                                    
////                  #ifdef AccelSensitivity_4G
////                  dmpGetLinearAccel_4G(&aaReal, &aa, &gravity);
////                  #endif
////                  
////                  #ifdef AccelSensitivity_8G
////                  dmpGetLinearAccel_8G(&aaReal, &aa, &gravity);
////                  #endif            
////                        
////              mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//              
//              for (int i=0;i<=NumOfSamples;i++)
//              {
//                AccelX[i]=AccelX[i+1];
//                AccelY[i]=AccelY[i+1];
//                AccelZ[i]=AccelZ[i+1];
//              }
//              AccelX[NumOfSamples]=aaWorld.x;
//              AccelY[NumOfSamples]=aaWorld.y;
//              AccelZ[NumOfSamples]=aaWorld.z; 
//  
//              int16_t aaWorldX_sum=0;
//              int16_t aaWorldY_sum=0;
//              int16_t aaWorldZ_sum=0;
//              
//              for (int i=0;i<NumOfSamples;i++)
//              {
//                aaWorldX_sum+=AccelX[i];
//                aaWorldY_sum+=AccelY[i];
//                aaWorldZ_sum+=AccelZ[i];
//              }
//              ///
//                  #ifdef AccelSensitivity_2G
//                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/8192;
//                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/8192;
//                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/8192;
//                  #endif
//                  
//                  /// For 4G sensitivity
//                  #ifdef AccelSensitivity_4G
//                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/4096;
//                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/4096;
//                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/4096;
//                  #endif  
//                  
//                  /// For 8G sensitivity
//                  #ifdef AccelSensitivity_8G
//                  AAccel->x= (float) aaWorldX_sum/NumOfSamples*9.81/2048.0;
//                  AAccel->y= (float) aaWorldY_sum/NumOfSamples*9.81/2048.0;
//                  AAccel->z= (float) aaWorldZ_sum/NumOfSamples*9.81/2048.0;
//                  #endif
//                        
//            
//            }
//  
//  }

// ================================================================
// ===                     SPEED CALCULATION                    ===
// ================================================================

void speed_calc(Speed *spd,AvgAccel accel, float delta_t)
 {
  
  spd->x = spd->x + accel.x*delta_t/1000.0;
  
  spd->y = spd->y + accel.y*delta_t/1000.0;
  
//  spd->z = spd->z + accel.z*delta_t/1000.0;
  
 }





// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


void dmpDataReady() {
    mpuInterrupt = true;
    sample_time=millis();
}




// ================================================================
// ===               Compensate Gravity    4G Range             ===
// ================================================================

uint8_t dmpGetLinearAccel_4G(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) {
 // get rid of the gravity component (+1g = +4096 in standard DMP FIFO packet, Range is 4g)
    v -> x = vRaw -> x - gravity -> x*4096;
    v -> y = vRaw -> y - gravity -> y*4096;
    v -> z = vRaw -> z - gravity -> z*4096;
    return 0;
}



// ================================================================
// ===               Compensate Gravity    8G Range             ===
// ================================================================
uint8_t dmpGetLinearAccel_8G(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity) 
{
 // get rid of the gravity component (+1g = +2048 in standard DMP FIFO packet, Range is 8g)
    v -> x = vRaw -> x - gravity -> x*2048;
    v -> y = vRaw -> y - gravity -> y*2048;
    v -> z = vRaw -> z - gravity -> z*2048;
    return 0;
}
// This function is used to compute the motor's duty (speed) 
// which will be prportional to peak foot speed
int motor_duty(float peak_foot_spd)
{
  return 8*peak_foot_spd+68;
  }


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
      // ================================================================
      // ===                      Motor SETUP                         ===
      // ================================================================
      // declare pin 10 to be an output:
      pinMode(10, OUTPUT);
      setPwmFrequency(10, 8);           //  3921.16 Hz
    
    
       pinMode(9, OUTPUT);
      setPwmFrequency(9, 8);           //  3921.16 Hz

      
  InitMotor();
      
  HM10MasterSerialInit();
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    // initialize SWSerial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    SWSerial.begin(115200);
    while (!SWSerial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    SWSerial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    SWSerial.println(F("Testing device connections..."));
    SWSerial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    SWSerial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (SWSerial.available() && SWSerial.read()); // empty buffer
    while (!SWSerial.available());                 // wait for data
    while (SWSerial.available() && SWSerial.read()); // empty buffer again

    // load and configure the DMP
    SWSerial.println(F("Initializing DMP..."));
//            mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
//            DEBUG_PRINTLN(F("Setting accelerometer sensitivity to +/- 4..."));
//            I2Cdev::writeByte(0x68, MPU6050_RA_ACCEL_CONFIG, 0x01);

    devStatus = mpu.dmpInitialize();
    
    // config 4g sensitivity
    
    #ifdef AccelSensitivity_4G
    mpu.setFullScaleAccelRange(1);  //0 = +/- 2g | 1 = +/- 4g | 2 = +/- 8g | 3 =  +/- 16g 
    #endif

    #ifdef AccelSensitivity_8G
    mpu.setFullScaleAccelRange(2);  //0 = +/- 2g | 1 = +/- 4g | 2 = +/- 8g | 3 =  +/- 16g 
    #endif
//Your MPU6050 should be placed in horizontal position, with package letters facing up. 
//Sensor readings with offsets:  4 1 16374 -1  0 0
//Your offsets: -2182 -1810 4667  74  -4  -60
//
//Data is printed as: acelX acelY acelZ giroX giroY giroZ
//Check that your sensor readings are close to 0 0 16384 0 0 0

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(26);
    mpu.setYGyroOffset(-7);
    mpu.setZGyroOffset(-23);
    
    mpu.setXAccelOffset(-1128);
    mpu.setYAccelOffset(-2239);
    mpu.setZAccelOffset(867);
    //    make sure it worked (returns 0 if so)
    //
    //    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        SWSerial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
      
        // enable Arduino interrupt detection
        SWSerial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        SWSerial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else 
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        SWSerial.print(F("DMP Initialization failed (code "));
        SWSerial.print(devStatus);
        SWSerial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
    SWSerial.println("I am Master!");
    
    //  =======================================================
    // set the data rate for the SoftwareSWSerial port
    //  =======================================================


}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    // Items that can trigger an interrupt are (1) Clock generator locked to new reference oscillator (used when switching clock 
    // sources); (2) new data is available to be read (from the FIFO and Data registers); (3) accelerometer event
    // interrupts; and (4) the MPU-60X0 did not receive an acknowledge from an auxiliary sensor on the secondary I2C bus
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here

    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //delay(200);
        SWSerial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } 
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    

    
            #ifdef OUTPUT_READABLE_WORLDACCEL
            if( time1<=5000)
            {
              time1=sample_time;
            }
            
            // wait 5s to prevent the noise of the accel sensor at start up
            
            if (time1>5000)
            {
                // display initial world-frame acceleration, adjusted to remove gravity
                // and rotated based on known orientation from quaternion
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                
                #ifdef AccelSensitivity_2G
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                #endif
                
                #ifdef AccelSensitivity_4G
                dmpGetLinearAccel_4G(&aaReal, &aa, &gravity);
                #endif
    
                #ifdef AccelSensitivity_8G
                dmpGetLinearAccel_8G(&aaReal, &aa, &gravity);
                #endif
                
                // calculate the World Accel 
                
                mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

                // save old world accel value to compute the rate of change
                
                AVAWorld1.x= AVAWorld.x;
                AVAWorld1.y= AVAWorld.x;
                AVAWorld1.z= AVAWorld.x;
                
                // current value of  world accel
                
                AVAWorld.x= (float) aaWorld.x*9.81/2048.0;
                AVAWorld.y= (float) aaWorld.y*9.81/2048.0;
                AVAWorld.z= (float) aaWorld.z*9.81/2048.0;
                
                // Reset the accel to zero if it is small 
                // Store the "NumSamplesToSetZero" valued=s of acceleration, if they are all zeros, we will reset the speed to zero
                // EX: AVAWorldMagSeries[0:1]
                // the "1st sample" (run1==1) will be stored in AVAWorldMagSeries[1];
                
                // From the second sample (run1!=1), we will store as:
                //            "1st sample" stored in AVAWorldMagSeries[0];
                //            "second sample" stored in AVAWorldMagSeries[1];
                
                if(run1==1)
                {

                    if (absolute(AVAWorld.x)<AccelMagThreshold)
                    {
                      AVAWorldMagSeries[NumSamplesToSetZero-1]=0;
                    }
                    else
                    {
                      AVAWorldMagSeries[NumSamplesToSetZero-1]= absolute(AVAWorld.x); 
                    }
                    time1=sample_time;
                    run1++;
                }
                else
                {
                    for(int ii=0;ii<NumSamplesToSetZero-1;ii++)
                      {
                        AVAWorldMagSeries[ii]= AVAWorldMagSeries[ii+1];
                      }
                   
                    if (absolute(AVAWorld.x)<AccelMagThreshold)
                      {
                        AVAWorldMagSeries[NumSamplesToSetZero-1]=0;
                      }
                    else
                    {
                      AVAWorldMagSeries[NumSamplesToSetZero-1]= absolute(AVAWorld.x); 
                    }
                    
                    time_old=time1;
                    time1=sample_time;
                }  
                        // compute the rate of change (RoCh) of acceleration to tackle with wrong speed reset bug       
                        delta_t=(time1-time_old);
                        RoCh=(AVAWorld.x-AVAWorld1.x)*1000.0/(float)delta_t;
//                        SWSerial.print(RoCh);
//                        SWSerial.print(",");
                        //==================================================================//
                        //==============    RESET SPEED TO ZERO IF NECESSARY ===============//
                        //==================================================================//
                        SumMagAccel=0;
                        
                        for(int ii=0;ii<NumSamplesToSetZero;ii++)
                        {
                           SumMagAccel+=AVAWorldMagSeries[ii];
                        }
                          
                        //==================================================================//
                        // store the speed values to detect the begin of each step 
                        // in order to capture half_step_time which will be used for the timing of motor
                        //==================================================================//
                        Spds[0].x=Spds[1].x;
                        Spds[0].y=Spds[1].y;
                        Spds[1].x=Spds[2].x;
                        Spds[1].y=Spds[2].y;
                        Spds[2].x=Spds[3].x;
                        Spds[2].y=Spds[3].y;
                        Spds[3].x=spd[1].x;
                        Spds[3].y=spd[1].y;
                        
                        // Use the speed_calc to calculate the user speed from the acceleration
                        speed_calc(&spd[1],AVAWorld, delta_t);
                                              
                        if(SumMagAccel==0 && absolute(RoCh)<RoChThreshold)// add abs_x<0.8 to prevent wrong speed reset :((
                        {
                          // we should realize the peak value and do not reset the speed to zero
//                          SWSerial.print("here,");
                          spd[1].x=0;
                          spd[1].y=0;
                          spd[1].z=0; 
//                          AVAWorld.x=0;
//                          AVAWorld.y=0;
//                          AVAWorld.z=0; 
                          peak_speed=0; 
                        }
                        //  ==================================================================//
                        //  Catch the peak speed value 
                        //  ==================================================================//
                        //  we must change something here to capture the time correctly
                        //  one sample =0; then we have 4 samples !=0 => begin the step 
                        if (Spds[0].x==0 && Spds[1].x!=0 && Spds[2].x!=0 && Spds[3].x!=0 & spd[1].x!=0)
                        {
                          SWSerial.print("Here,");
                          SWSerial.print(spd[0].x);
                          SWSerial.println(spd[1].x);
                          step_start_time=sample_time;
                         }
                        abs_x=absolute(spd[1].x);  
//                      per our test, the peak value of normal walk will never drop below 0.8
                        if (abs_x>0.8)
                        peak_speed=max(peak_speed,abs_x); // we have to use our own absolute function because built-in abs() returns int value
                        
                        //==================================================================//
                        //        CATCH PEAK SPEED VALUES
                        //        Modify the code to detect the peak of 4 steps
                        //        current speed < previous speed  that means we finish with the 1st peak
                        //==================================================================//  
                        //==================================================================//    
                          
                        // peak_speed>0.5 to make sure that is an actual peak .
                        //  absolute(spd[1].x)!=0 to tackle with wrong "reset to zero" when we reset the speed
                        if (absolute(spd[1].x) < peak_speed && peak_speed>0.5 && absolute(spd[1].x)!=0 ) //the speed value is going down.
                        {

//                            SWSerial.println("T"+half_step_time); 
                            // j variable is used here because the foot speed will continue to  go down after the peak value, we only care about the 1st value
                            if (!j)// j==0
                            {
                                  peak_count++;                               
                                  step_peak_time=sample_time;
                                  half_step_time=step_peak_time-step_start_time;
                                  SWSerial.println("H");
                                  SWSerial.println(half_step_time);
                                  adapttomyself=true;
                                  // transmit TXAdaptedSignal to signal the other foot to adapt his motor speed
                                  Serial.write(TXAdaptedSignal);// TXAdaptedSignal=2, we can choose any encoded value                                  
//                                  Serial.write(TXAdaptedSignal);// TXAdaptedSignal=2, we can choose any encoded value     
//                                  Serial.write(TXAdaptedSignal);// TXAdaptedSignal=2, we can choose any encoded value
                                  // update the values of 5 latest peak speeds
                                  peak_speeds[0]=peak_speeds[1];
                                  peak_speeds[1]=peak_speeds[2];
                                  peak_speeds[2]=peak_speeds[3];
                                  peak_speeds[3]=peak_speeds[4]; 
                                  peak_speeds[4]=peak_speed;
                                  
                                  avg_peak_speed=(peak_speeds[0]+peak_speeds[1]+peak_speeds[2]+peak_speeds[3])/4;
                                  
                                  //  tend to reduce the user's speed
                                  ratio=peak_speeds[4]/avg_peak_speed;
                                  SWSerial.println(ratio);
                                  // this is at Master side
                                  if (ratio<0.92 && ratio >=turnoff_Ratio)
                                  {
                                    //send data by bluetooth
                                    Serial.write((byte)0x00);
//                                    Serial.write((byte)0x00);
//                                    Serial.write((byte)0x00);
//                                    SWSerial.println("Se1M");
                                  }
                                  // transmit "1" over BLE to signal the other foot that we are stopping. 
                                  // As a result,  "stopbyOther" flag  of the other foot will be enable.
                                  else if(ratio<turnoff_Ratio && ratio>0)
                                  {
                                    //send data by bluetooth
                                    Serial.write(1);
//                                    Serial.write(1);
//                                    Serial.write(1);
//                                    SWSerial.println("Se2M");
                                  }

                            }
                            j=1;
                         }
                         // reset variable "j" so that we can capture the next peak foot speed.
                         else
                         {
                          j=0;
                         }
                         
                        //===================================================================
                        // This code is designed for Starting Mechanism 
                        //===================================================================
                        //===================================================================
                        // FOR the very 1ST FOOT STEP    
                        //===================================================================  
                        
                        //  Relation between motor speed (duty) and peak foot speed is chosen as duty=8*peak_speed+68     (we can choose other function).
                                 
                        //  We will increase the motor speed in the duration of "half_step_time/2" (starting from the moment we capture peak foot speed.
                        
                        //  startup_safe_duty=90 is for the user's security, it will prevent the motor from rotating too fast.

                        //  MtorIsMoving variable is used to differentiate the 1st step, it could be left foot or right foot.
                        
                        if( !MtorIsMoving && peak_speeds[4]>0 && peak_speeds[3]==0  &&  (sample_time-step_peak_time) <= half_step_time/2) // 1st step
                        {                          
                          duty=motor_duty(peak_speeds[4])*(sample_time-step_peak_time)/(half_step_time/2); // the motor speed (duty) will be proportional to the peak foot speed
                          // we do not to move too fast at the beginning
                          if(duty>startup_safe_duty)
                          {
                            duty=startup_safe_duty;
                            }
                          // motor  already stopped when duty < turnoff_threshold (this depends on the ESC firmware)                            
                          if(duty>turnoff_threshold)
                          {
                            // send signal to the ESC to control the motors
                           duty=duty>=non_stop_threshold?duty:non_stop_threshold;

                            analogWrite(10,duty);
                            analogWrite(9,duty) ;
                            // send to bluetooth which is connected to HW Serial
                            Serial.write(duty); 
                          }
                          // Serial print for debugging
                          SWSerial.print("dt:");
                          SWSerial.println(duty);   
                          // variable duty_set will be used to store the motor's current spd. View "SPEED CHANGE BEHAVIOUR" section for better understanding                      
                          duty_set=duty;
                        }

                        //==================================================================  //
                        //==============              SWSerial Print           ===============//
                        //==================================================================  //
                        
                        SWSerial.print(sample_time); 
                        SWSerial.print(",");                         
//                        SWSerial.print(AVAWorld.x); 
//                        SWSerial.print(",");             
                        SWSerial.print(spd[1].x);
                        SWSerial.print(",");                         
                        
                        SWSerial.println(spd[1].y);
//                        SWSerial.print(",");
//                        SWSerial.print(1].x);
//                        SWSerial.print(",");
//                        SWSerial.print(peak_speeds[0]);
//                        SWSerial.print(",");
//                        SWSerial.print(peak_speeds[1]);
//                        SWSerial.print(",");
//                        SWSerial.print(peak_speeds[2]);
//                        SWSerial.print(",");
//                        SWSerial.print(peak_speeds[3]); 
//                        SWSerial.print(",");
//                        SWSerial.println(peak_speeds[4]); 
                                        

                //==================================================================//
                //     CODE FOR SECURITY: turn off motors when lose BLE connection
                //==================================================================//
                // Exchange the  PilotSignal with other shoe. 
                // if do not receive PilotSignal for more than 650ms, then we lost the Bluetooth connection.
                // turn off the motor when we lose the Bluetooth connection.
                if(millis()-pilot_send_time>300) // send pilot signal every 300ms
                {
                  pilot_send_time=millis();
                  Serial.write(PilotSignal);
                  SWSerial.print("PL");
                  SWSerial.print(PilotSignal);
                }                                                 
                        
              }
            //==================================================================//
            //==============       BLE SoftwareSWSerial Print      ===============//
            //==================================================================//  
            //            Serial.println(AVAWorld.x);
              #endif

            }
        
        
        //  This stopping mechanism should be reviewed again
        //  RX_Data_BLE==0: slave ratio <0.9, >turnoff_Ratio
        //  RX_Data_BLE==1: slave ratio <turnoff_Ratio
        if((ratio<turnoff_Ratio))  // Stop by myself
        {
//          half_step_time=step_peak_time-step_start_time;
//          duty=90*peak_speeds[4]*(step_peak_time+half_step_time-sample_time)/(half_step_time); // the motor speed will proportional to the peak foot speed

          // gradually stop the motor, begin from the time we capture the peak value and the duration of this process is "half_step_time".

          if(step_peak_time+half_step_time>sample_time) // sample_time < step_peak_time+half_step_time
          {
            gradualStopDuty=duty*(step_peak_time+half_step_time-sample_time)/(half_step_time);
          }
          SWSerial.print("STbymyself");
          if (gradualStopDuty>turnoff_threshold)
          {
            analogWrite(10,gradualStopDuty);
            analogWrite(9,gradualStopDuty);
            
            Serial.write(gradualStopDuty);// signal the Slave to stop
            SWSerial.println(gradualStopDuty);
          }
        }
        else if(stopbyOther)// stop by other foot if receive 1 from BLE (RX_Data_BLE==1) <=> slave ratio <turnoff_Ratio
        {
            SWSerial.print("STbyother:");
            if (RX_Data_BLE>turnoff_threshold)
            {
              SWSerial.println((int)RX_Data_BLE);       
              analogWrite(10,RX_Data_BLE);
              analogWrite(9,RX_Data_BLE);
            }
          }

         // ========================================
         //         SPEED CHANGE BEHAVIOUR. 
         // ========================================
         //   the motor will adapt to the new_duty which is given by new_duty=8*peak_speeds[4]+68 (again, we can choose the function)
         
         //   We will change the motor speed in the duration of "half_step_time/2" (starting from the moment we capture new peak foot speed)
         
         //   value 110 is for the user's safety, it will prevent the motor from rotating too fast
         
         //   "adapttomyself" is a variable used to adapt the motor speed to the its foot speed (peak foot speed from this program, not from bluetooth BLE).
         
         if(millis()-pilot_receive_time<650)
         {
          if(adapttomyself && !stopbyOther && ratio >turnoff_Ratio && MtorIsMoving)// MtorIsMoving  is used to isolate this code from 1st foot step
          {
              new_duty=motor_duty(peak_speeds[4]); 
              SWSerial.println(duty_set);
              
              //  y decrease/increase the motor's speed, we choose a linear function here.
              
              if(duty_set<safe_duty_threshold && (sample_time-step_peak_time) <= half_step_time/2)//decrease upto the "new_duty" value
              {
                duty_set=(int)(duty+(new_duty-duty)*(sample_time-step_peak_time)/(half_step_time/2));
                SWSerial.print("ds");
                SWSerial.println(duty_set);
//                SWSerial.println(new_duty);
//                SWSerial.println(duty);
//                SWSerial.println(sample_time);
//                SWSerial.println(step_peak_time);
//                SWSerial.println(half_step_time);
                // signal the Slave to decrease speed (by Bluetooth)
                duty_set=duty_set>=non_stop_threshold?duty_set:non_stop_threshold;
                
                Serial.write(duty_set);
                // send to signal to ESC motor controller                                         
                analogWrite(10,duty_set);
                analogWrite(9,duty_set);
              }
              else if ((sample_time-step_peak_time) > half_step_time/2)
              {
                duty=duty_set;
                }
            }          
         }
         else // if lose the BLE connection, we will stop our motor
         { 
            SWSerial.print("LST");
            analogWrite(10,5);
            analogWrite(9,5);
            lost_connection=true;
          }

}


//This is HW interrupt of the Serial,  the blue tooth module will be connected to HW Serial to tackle the noise at the BLE receiver.

float absolute(float x)
{
  if (x>0)
    return x;
  else
    return -x;
}

















  
