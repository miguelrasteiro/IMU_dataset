# IMU_dataset
A matlab dataset of various tests of inertial and magnetic sensors using an industrial robot as ground-truth.

# Considerations of the dataset and tests
This repository intends to make available a dataset collected during an evaluation phase of a few fusion algorithms. 

Allowing to compare the performance of fusion algorithms in a harsh dynamic environment with no prior knowledge of the magnetic field disturbances or linear accelerations caused by motion against some reference sensors available on the market.

The tests were carried out using an industrial robot that could retrieve the IMU unit orientation and position at 15 Hz, establishing the ground-truth. 

All sensors signals were sampled at 100 Hz and interfaced to a PC via a custom wireless protocol between two MRF24J40 Microchip modules.

Both signals were synchronized in matlab time.

The tests were performed over 4 different paths at 6 different velocities with 8 different sensors. Run TestScript.m for more details.

6 different velocities: 
- 150  mm/s (safe mode) 
- 300  mm/s
- 500  mm/s
- 800  mm/s
- 1000 mm/s
- 1500 mm/s

4 different paths:  
- path1 (yaw rotation - pitch rotation roll - rotation)
- path2 (linear movements)
- path3 (complex rotations)
- path4 (complex rotations)

4 sensors retrieving raw data: 
- LSM9DS0 (gyro+acc+mag) 100Hz
- MPU9150 (gyro+acc+mag) 100Hz
- MPU6500 + RM3100 (gyro+acc+mag) 100Hz
- MPU6050 + RM3100(gyro+acc+mag) 100Hz

4 reference sensors retrieving orientation: 
- PNI SENTRAL M&M Blue (gyro+acc+mag) 100Hz
- XSENS MTi-30 (gyro+acc+mag) 100Hz
- MPU6050 (gyro+acc) 100Hz
- MPU6050 (gyro+acc) 100Hz

# Notes
Reference sensors/algorithms used:
- PNI SENTRAL M&M Blue Module runs a stateof-the-art patented KF algorithm performing continuous hard and soft-iron disturbance calibration and magnetic anomalies compensation; 
- XSENS MTi-30 series runs a proprietary XSens Kalman Filter; 
- MPU6500DMP and MPU6050DMP run an Invensense MotionFusion
Algorithm (IMFA) with 6-axes (no usage of magnetometer data) integration.
The tests were performed with four different sets of sensors:

Tested sensors:
- 9-axes integrated MPU9150 from Invensense; 
- 9-axes integrated LSM9DS0 from STMicroeletronics; 
- 9-axes integration with a 6-axes MPU6050 (Gyroscope+Accelerometer) and the 3-axes magnetic-inductive RM3100 (EVb) from PNICorp; 
- 9-axes integration with a 6-axes MPU6500 (Gyroscope+Accelerometer) and the 3-axes RM3100 EVb. 
