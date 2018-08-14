# kalman_filter_matlab
Implementation of Discrete Kalman Filter in Matlab/Octave with simple examples

This repository demonstrates the implementation of Kalman filter with simple examples in Matlab/Octave.
The implementational steps are based on the paper entitled "An Introduction to the Kalman Filter" by Greg Welch and Gary Bishop.
The paper can be downloaded at https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf .


The "simpleKalmanFilter.m" example is based on the voltage reading error described in the above mentioned paper.
However, the resultant graphs could be a little bit different from the provided ones in the paper because of the random generator used in this particular implementation.


The "KF_train_const_speed.m" and "KF_train_sys_input.m" examples are based on the one-dimentional train tracking problem.
The problem is described in details in the lecture note entitled "Understanding the Basis of the Kalman Filter Via a Simple and Intuitive Derivation" by Ramsey Faragher.
The lecture note can be downloaded at https://courses.engr.illinois.edu/ece420/fa2017/UnderstandingKalmanFilter.pdf .
