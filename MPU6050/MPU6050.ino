/* =====================================================
 * Example of Invensense MPU 6050 (depreciated product) 
 *     Copyright (C) 2017 James (Aaron) Crabb
 *     You should have received a copy of the GNU General Public License
 *     along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 *  This is example is part of 'Thinking with Reference Frames'
 *  For full documentation please visit 
 *  https://github.com/jacrabb/Thinking-with-Reference-Frames
 *
 * The MPU6050 has gained wide adoption and is one of the best
 * supported devices in the open hardware ecosystem.
 * However, I warn you that many people experience problems
 * using its more advanced features.  Primarily, I think this 
 * problem is due to improper reads from the device's FIFO buffer.
 * Personally, I have had good experiences using the device with
 * fast (120MHz) MPUs using the (arduino compatible) code contained
 * in this example.
 * I have relied on the MIT liscensed jrowberg library wich is great
 * but also somewhat lacking.
 *  https://github.com/jrowberg/i2cdevlib
 * Also, I stole some insight from a StackOverflow post, but I have
 * lost the link to provide correct attribution.
 * The example below uses two MPU6050's with different I2C addresses
 * and different interrupt pins.
 * The chip's onboard DMP software for FIFO, interrupt, and initial
 * orientation calculation (which is a low-pass filter technique 
 * to isolate gravity and provides orientation as a quaternion).
 * The data output of this examaple is designed to work with 
  * https://github.com/jacrabb/IMUDataVis.Processing
 */
 
#define PRINTING

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// in "MPU6050_6Axis_MotionApps20.h" 
// line 305:  0x02,   0x16,   0x02,   0x00, 0x01
// D_0_22 inv_set_fifo_rate 100Hz
#include "Wire.h"

// AD0 low  = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu[] = {MPU6050(0x68), MPU6050(0x69)};


#define INTERRUPT_PIN  A2
#define INTERRUPT_PIN2 A3
#define LED_PIN       13


uint8_t mpuIntStatus[2];   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
volatile bool mpuInterrupt[2] = {false,false};     // indicates whether MPU interrupt pin has gone high

uint8_t fifoBuffer[2][64]; // FIFO storage buffer

// if there are changed to fixed point, you should change a memcpy on gesture()
// if float is not 16bits, then you MUST change the memcpy
Quaternion q[2];
Quaternion16 qFixedP[2];
VectorInt16 aa[2], aaFilter[2];
VectorInt16 aaReal[2], aaRealWorld[2];
VectorInt16 g[2], gFilter[2];
VectorInt16 gWorld[2];
VectorFloat /*VectorInt16*/ gravity[2];
float euler[2][3];
float ypr[2][3];
float Yaw[2], Pitch[2], Roll[2];


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

// PacketSize = 42; refference in MPU6050_6Axis_MotionApps20.h Line 527
// FIFO Buffer Size = 1024
// 20*42=840 leaving us with  2 Packets (out of a total of 24 packets) left before we overflow.
// 22*42=924
// 23*42=966
// 24*42=1008
// 25*42=1050 !!! more than 1024, so not possible
#define MaxPackets packetSize * 24

// inturrupt handlers
//  see helper function for more
void dmpDataReady() {
  interruptHelper(0);
}
void dmpDataReady2() {
  interruptHelper(1);
}


// ================================================================
// ===                       ARDUINO SETUP                      ===
// ================================================================
void setup(){
  #ifdef PRINTING 
   setprint(2000000);
  #endif

  //mpu[0] = new MPU6050(0x68);
  //mpu[1] = new MPU6050(0x69);
  // AD0 low  = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
  // AD0 high = 0x69
  
  pinMode(INTERRUPT_PIN, INPUT);
  pinMode(INTERRUPT_PIN2, INPUT);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  i2cSetup();

  MPU6050Setup();

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);  // -aac from the sparkfun demo
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN2), dmpDataReady2, FALLING);
  
}

    
// ================================================================
// ===                          MAIN                            ===
// ================================================================
void loop(){
  //order of these compared to rising/falling inerrupt seems important
    if(mpuInterrupt[1]){
      arrangeData(1);
      getYPR(1);
        
      mpuInterrupt[1] = false;
      #ifdef PRINTING 
       //printMPU(1);
      #endif
    }
    
    if(mpuInterrupt[0]){
      // should we block interrupts while we fetch the basic data, but then allow during computation ?
      arrangeData(0);
      getYPR(0);
        
      mpuInterrupt[0] = false;
      #ifdef PRINTING 
       printMPU(0);
      #endif
    }

  }

/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

