
void setprint(int sbaud){
 Serial.begin(sbaud); //115200
 while (!Serial);
}
// ANYTHING turns output off 
//  (still alows debug messages)
#define P_ANYTHING
#define P_ACCEL
#define P_GYRO
#define P_ORIENTATION
//#define P_YPRRAW
#define P_ACCELFILTER
//#define P_ACCELREAL
#define P_ACCELREALWORLD
#define P_GESTURE
#define P_GYROWORLD
//#define P_GRAV
//#define P_QUAT

void printMPU(int i){
  #ifdef P_ANYTHING
  if(!i) Serial.print(("//\t")); // 0
  else Serial.print(("\\\\\t")); // 1
  
    // accel
    #ifdef P_ACCEL
                          Serial.print(aa[i].x);
    Serial.print(("\t")); Serial.print(aa[i].y);
    Serial.print(("\t")); Serial.print(aa[i].z);
    Serial.print(("\t"));
    #endif
    
    // gyro
    #ifdef P_GYRO
                          Serial.print(g[i].x);
    Serial.print(("\t")); Serial.print(g[i].y);
    Serial.print(("\t")); Serial.print(g[i].z);
    Serial.print(("\t")); 
    #endif
    
    // final ypr orientation
    #ifdef P_ORIENTATION
                          Serial.print(Roll[i]);
    Serial.print(("\t")); Serial.print(Pitch[i]);
    Serial.print(("\t")); Serial.print(Yaw[i]);
    Serial.print(("\t"));
    #endif
    
    // uncorrected ypr orientation
    #ifdef P_YPRRAW
                          Serial.print(ypr[i][1]);
    Serial.print(("\t")); Serial.print(ypr[i][2]);
    Serial.print(("\t")); Serial.print(ypr[i][0]);
    Serial.print(("\t"));
    #endif
    
    // accel lp filtered
    #ifdef P_ACCELFILTER
                          Serial.print(aaFilter[i].x);
    Serial.print(("\t")); Serial.print(aaFilter[i].y);
    Serial.print(("\t")); Serial.print(aaFilter[i].z);
    Serial.print(("\t"));
    #endif
    
    // accel without gravity in local frame
    #ifdef P_ACCELREAL
                          Serial.print(aaReal[i].x);
    Serial.print(("\t")); Serial.print(aaReal[i].y);
    Serial.print(("\t")); Serial.print(aaReal[i].z);
    Serial.print(("\t"));
    #endif
    
    // accel without gravity in world frame
    #ifdef P_ACCELREALWORLD
                          Serial.print(aaRealWorld[i].x);
    Serial.print(("\t")); Serial.print(aaRealWorld[i].y);
    Serial.print(("\t")); Serial.print(aaRealWorld[i].z);
    Serial.print(("\t"));
    #endif
    
    // gyro lp filtered
    #ifdef P_GESTURE
                          Serial.print(gesturePointer);
    Serial.print(("\t")); Serial.print((i == 0) ? gestureArray[gesturePointer] : 0);
    Serial.print(("\t")); Serial.print((i == 0) ? gestureTemplate[gesturePointer] : 0);
    Serial.print(("\t"));
    #endif
    
    // gyro in world frame
    #ifdef P_GYROWORLD
                          Serial.print(gWorld[i].x);
    Serial.print(("\t")); Serial.print(gWorld[i].y);
    Serial.print(("\t")); Serial.print(gWorld[i].z);
    Serial.print(("\t"));
    #endif
    
    // gravity unit(?) vector
    #ifdef P_GRAV
                          Serial.print(gravity[i].x);
    Serial.print(("\t")); Serial.print(gravity[i].y); 
    Serial.print(("\t")); Serial.print(gravity[i].z);
    Serial.print(("\t"));
    #endif
    
    // the quaternion orientation
    #ifdef P_QUAT
                          Serial.print(q[i].x);
    Serial.print(("\t")); Serial.print(q[i].y);
    Serial.print(("\t")); Serial.print(q[i].z);
    Serial.print(("\t")); Serial.print(q[i].w);
                         
    #endif

    
    Serial.println();
    #endif
}
