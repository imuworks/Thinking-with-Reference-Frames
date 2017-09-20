# Thinking-with-Reference-Frames
Advanced IMU examples and libraries based on things I've learned working in inertial navigation, wearables, and biometrics.

## Contents
Currently I have only committed one code example for the Invensense MPU6050.  This example is an Arduino sketch and uses the 'standard' open source MPU6050 library.  It demonstrates an easy way to support multiple IMUs (the MPU6050 is limited to 2 instances because the hardware only supports two I2C addresses via the ADO pin).
This code will hopefully be a basis for demonstrating more advanced multi-reference frame examples.

Additionally, I have started a discussion of IMU reference frames.  My intent is for this to become a more advanced tutorial on IMU concepts.
