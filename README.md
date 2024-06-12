# ACE 3.0 Development Board Code

Work is being done to communicate with all of the sensors on the board (BMP581, BMI088, BMM350). In the future, all sensors will be read and a kalman filter will fuse the data into a 3d state estimation. Also, SD card slot will be used to log state data for view after flights. FreeRTOS is used to coordinate work between tasks.
