// ================================================================
// ===                      MPU SETUP Items                     ===
// ================================================================

static const void DMPOffsets(){

  // Please calibrate and find your own offset values.
  //                           ax       ay     az     gx    gy   gz
  int MPUOffsets[2][6]  ={ {  -1829,    33,   1818,  169,  -5,   28},
                           {  -4468,  -2857,  1614,  144,   1,   24} };
  
  mpu[0].setXAccelOffset(MPUOffsets[0][0]);
  mpu[0].setYAccelOffset(MPUOffsets[0][1]);
  mpu[0].setZAccelOffset(MPUOffsets[0][2]);
  mpu[0].setXGyroOffset(MPUOffsets[0][3]);
  mpu[0].setYGyroOffset(MPUOffsets[0][4]);
  mpu[0].setZGyroOffset(MPUOffsets[0][5]);
  
  mpu[1].setXAccelOffset(MPUOffsets[1][0]);
  mpu[1].setYAccelOffset(MPUOffsets[1][1]);
  mpu[1].setZAccelOffset(MPUOffsets[1][2]);
  mpu[1].setXGyroOffset(MPUOffsets[1][3]);
  mpu[1].setYGyroOffset(MPUOffsets[1][4]);
  mpu[1].setZGyroOffset(MPUOffsets[1][5]);
}


void MPU6050Setup(){
  digitalWrite(LED_PIN, HIGH);    // pulse from here
  uint8_t devStatus;
  //basic init
  mpu[0].initialize(); 
  mpu[1].initialize();

  //load config
  devStatus = mpu[0].dmpInitialize(); 
  devStatus += mpu[1].dmpInitialize();

  // try forever.
  if (devStatus != 0) {
    #ifdef PRINTING
    Serial.print("ERROR\nDev Status is ");
    Serial.print(devStatus);
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    #endif
    delay(1000);
    MPU6050Setup();
  }
  digitalWrite(LED_PIN, LOW);   // to here

  DMPOffsets();

  // set the DMP EN bit
  mpu[0].setDMPEnabled(true);
  mpu[1].setDMPEnabled(true);

  // 5Hz bandwidth on the builtin lfp
  mpu[0].setRate(9); // 1khz / (1 + 9) = 100Hz
  mpu[0].setDLPFMode(MPU6050_DLPF_BW_5);
  mpu[1].setRate(9);
  mpu[1].setDLPFMode(MPU6050_DLPF_BW_5);

  mpuIntStatus[0] = mpu[0].getIntStatus();
  mpuIntStatus[1] = mpu[1].getIntStatus();
  
  // get expected DMP packet size for later comparison
  packetSize = mpu[0].dmpGetFIFOPacketSize();
  packetSize = mpu[1].dmpGetFIFOPacketSize(); // yes, I call them both
  
  delay(450); // Let it Stabalize
  
  mpu[0].resetFIFO(); 
  mpu[1].resetFIFO(); // Clear fifo buffer
  
  mpu[0].getIntStatus(); 
  mpu[1].getIntStatus();
  
  mpuInterrupt[0] = false; // incase something tried to happen during init
  mpuInterrupt[1] = false; // which is impossible, as the inerrupt isn't attached yet
}

// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz) - code for AVR
  // you should check i2c bus w/ scope, as they're often asymptomatic even when improperly setup.
  Wire.setClock(800000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(800, true);
#endif
}
