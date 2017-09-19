
/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

void interruptHelper(int i) {
  //bail fast if we haven't even finished the last batch - guarantees an overflow
  // perhaps do something to tell the system that we're stepping on its toes?
  if(mpuInterrupt[i])
    return;
    
  /*cli();*/noInterrupts();
  const uint16_t fifoCount = mpu[i].getFIFOCount();

  // if count is more than our buffer OR less than a full packet OR not well-aligned
  if( (fifoCount > MaxPackets) || (fifoCount < packetSize) /*|| (fifoCount % packetSize) */ ) {
    // should throw an error, want to use port regs to make a light come on
        
    /*mpuIntStatus[i] = */mpu[i].getIntStatus(); // clear status
    // MPU6050_RA_INT_STATUS       0x3A
    //
    // Bit7, Bit6, Bit5, Bit4          , Bit3       , Bit2, Bit1, Bit0
    // ----, ----, ----, FIFO_OFLOW_INT, I2C_MST_INT, ----, ----, DATA_RDY_INT

    //Bit4 FIFO_OFLOW_INT: This bit automatically sets to 1 when a FIFO buffer overflow interrupt has been generated.
    //Bit3 I2C_MST_INT: This bit automatically sets to 1 when an I2C Master interrupt has been generated. For a list of I2C Master interrupts, please refer to Register 54.
    //Bit1 DATA_RDY_INT This bit automatically sets to 1 when a Data Ready interrupt is generated.
    //if (mpuIntStatus & 1<<4) //FIFO_OFLOW_INT
    //if (mpuIntStatus & 1<<3)//I2C_MST_INT
  }else {
    mpu[i].getFIFOBytes(fifoBuffer[i], fifoCount);
    mpuInterrupt[i] = true;
  }
  /*sei();*/interrupts(); //https://forum.arduino.cc/index.php?topic=457720.0
  mpu[i].resetFIFO(); // clear the buffer
}


// Orientation => gravity => accel san gravity => world ref accel
void arrangeData(int i){

  //get the basic 'raw' data
  arrangeData(i, true);

  // gets the orientation from the DMP unit
  mpu[i].dmpGetQuaternion(&q[i], fifoBuffer[i]);
  /*[] (Quaternion *q, const uint8_t* packet) {
    // TODO: accommodate different arrangements of sent data (ONLY default supported now)
    int16_t qI[4];
    qI[0] = ((packet[0] << 8) | packet[1]);
    qI[1] = ((packet[4] << 8) | packet[5]);
    qI[2] = ((packet[8] << 8) | packet[9]);
    qI[3] = ((packet[12] << 8) | packet[13]);
    
    q -> w = (float)qI[0] / 2; //16384.0f; // constant value? -aac TODO
    q -> x = (float)qI[1] / 2; //16384.0f;
    q -> y = (float)qI[2] / 2; //16384.0f;
    q -> z = (float)qI[3] / 2; //16384.0f;
  }(&q[i], fifoBuffer[i]);*/

  // gravity vector in local ref frame (unit vector?)
  // literal conversion from quat to vector?
  mpu[i].dmpGetGravity(&gravity[i], &q[i]);
  /*[] (VectorInt16 *v, Quaternion *q) {
    v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
    v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
    v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
  }(&gravity[i], &q[i]);*/

  // remove gravity from aacel values
  // It is not appropriate to call this 'linear'
  mpu[i].dmpGetLinearAccel(&aaReal[i], &aa[i], &gravity[i]);
/*[] (VectorInt16 *v, VectorInt16 *vRaw, VectorInt16 *gravity) {
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
    v -> x = vRaw -> x - gravity -> x; // *8192;  // constant value? -aac TODO
    v -> y = vRaw -> y - gravity -> y; // *8192;
    v -> z = vRaw -> z - gravity -> z; // *8192;
  }(&aaReal[i], &aa[i], &gravity[i]);*/

  // rotate accel values into world frame
  mpu[i].dmpGetLinearAccelInWorld(&aaRealWorld[i], &aaReal[i], &q[i]);

  // rotate gyro values into world frame
  [] (VectorInt16 *v, VectorInt16 *vReal, Quaternion *q) {
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v -> rotate(q);
  }(&gWorld[i], &g[i], &q[i]);
  

  // do we have a use for euler?
  //mpu[i].dmpGetEuler(&euler[i], q[i])
}
// get only 'raw' data
void arrangeData(int i, int basic){
  mpu[i].dmpGetAccel(&aa[i], fifoBuffer[i]);
  mpu[i].dmpGetGyro(&g[i], fifoBuffer[i]);
}

// orientation stuff


// Readable orientation, for when its needed
// MUST HAVE A CURRENT arrangeData()
void getYPR(int i){
  
  getYPR(i, true);
  
  Yaw[i] = (ypr[i][0] * 180 / M_PI);
  Pitch[i] = (ypr[i][1] * 180 / M_PI);
  Roll[i] = (ypr[i][2] * 180 / M_PI);
}
void getYPR(int i, int basic){
  // rotates around z with quat, x&y using gravity rotation matrix
  mpu[i].dmpGetYawPitchRoll(ypr[i], &q[i], &gravity[i]);
/*[] (float *data, Quaternion *q, VectorInt16 *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan((float)gravity -> x / (float)sqrt((float)gravity -> y*(float)gravity -> y + (float)gravity -> z*(float)gravity -> z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan((float)gravity -> y / (float)sqrt((float)gravity -> x*(float)gravity -> x + (float)gravity -> z*(float)gravity -> z));
  }(ypr[i], &q[i], &gravity[i]);*/
}


// fixedpoint low pass filter
// http://www.edn.com/design/systems-design/4320010/A-simple-software-lowpass-filter-suits-embedded-system-applications
// this is a strong lpf
//  the mpu built in filter does not seem as strong
inline void lpf(int i){
  #define lp_bit_shift 4
  static int32_t lp_reg[2][3]; // 3 axes x 2 sensors
  
  lp_reg[i][0] = lp_reg[i][0] - (lp_reg[i][0] >> lp_bit_shift) + g[i].x;
  gFilter[i].x = lp_reg[i][0] >> lp_bit_shift;
  lp_reg[i][1] = lp_reg[i][1] - (lp_reg[i][1] >> lp_bit_shift) + g[i].y;
  gFilter[i].y = lp_reg[i][1] >> lp_bit_shift;
  lp_reg[i][2] = lp_reg[i][2] - (lp_reg[i][2] >> lp_bit_shift) + g[i].z;
  gFilter[i].z = lp_reg[i][2] >> lp_bit_shift;
}


class Quaternion16 {
    public:
        int16_t w;
        int16_t x;
        int16_t y;
        int16_t z;
        
        Quaternion16() {
            w = 1;
            x = 0;
            y = 0;
            z = 0;
        }
        
        Quaternion16(int16_t nw, int16_t nx, int16_t ny, int16_t nz) {
            w = nw;
            x = nx;
            y = ny;
            z = nz;
        }

        Quaternion16 getProduct(Quaternion16 q) {
            // Quaternion multiplication is defined by:
            //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
            return Quaternion16(
                w*q.w - x*q.x - y*q.y - z*q.z,  // new w
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w); // new z
        }

        Quaternion16 getConjugate() {
            return Quaternion16(w, -x, -y, -z);
        }
        
        float getMagnitude() {
            return sqrt(w*w + x*x + y*y + z*z);
        }
        int16_t getMagnitude16() {
            return sqrt(w*w + x*x + y*y + z*z);
        }
        
        void normalize() {
            int16_t m = getMagnitude16();
            w /= m;
            x /= m;
            y /= m;
            z /= m;
        }
        
        Quaternion16 getNormalized() {
            Quaternion16 r(w, x, y, z);
            r.normalize();
            return r;
        }
};

