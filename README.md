# KalmanFilter3D
Kalman Filter implementation in 3D coordinate space

This is a Matlab implementation of a Kalman filter. The filter estimates the position and velocity of an object in 3D space.
Input and output vectors are arraged as [x, y, z, dx, dy, dz] where dx, dy, dz are relative changes in postion between observations.
Use testKalman.m to see how the filter function is called. 
