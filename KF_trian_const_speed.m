%--------------------------------------------------------------------------
% Name:            KF_train_const_speed.m
%
% Description:     A simple example of one-dimentional tracking problem
%                  using Kalman filter is demonstrated in the example. The 
%                  example is based on the description of the lecture note,
%                  which is available to download at the following link:
%                  https://courses.engr.illinois.edu/ece420/fa2017/UnderstandingKalmanFilter.pdf
%                  In particular, we are trying to estimate/track the best
%                  possible poistion and velocity of a train moving
%                  along a railway line in this example. As a control input  
%                  signal to the system, the train driver may apply brake or
%                  acceleration as required in order to change the dynamics of the 
%                  train movement. However, it is assume that the train is 
%                  moving with a constant speed and the system input signal
%                  is also constant (constant force) in this particular example
%                  for simplicity. 
%
%
% Author:          Cung Lian Sang
%                  salaikyonekyone12@gmail.com (or)
%                  csang@tecahfak.uni-bielefeld.de
%
% Date:            August 13, 2018
%--------------------------------------------------------------------------


close all; clear; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Initial Parameters & System Design %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

no_Samples = 100;        % no. of samples
init_vel = 12;          % initial constant velocity in m/s
delta_t = 0.1;          % update rate of the tracking (every xx seconds)
time_rate = delta_t: delta_t: delta_t * no_Samples; % length = no. of samples

Xtrue = zeros(2, no_Samples);    % Vector of true value (for graph)
init_pos = 10;                   % initial position

% System of equation for position and velocity.
Xtrue(1,:) = init_pos + init_vel .* time_rate;  % true train position
Xtrue(2,:) = init_vel;   % constant velocity 


% Previous state (a priori) estimated value 
Xk_prev = [];           % \cap{x_k}_minus symbol (a priori)

% Current state (a posteriori) estimated value (initial guess): 
Xk = [10;               % this is our initial guess (a posteriori)
     0.8 * init_vel];

% State Transition Matrix A represents the dynamics of the system: 
% see the system of equations and the detialed infos in the mentioned paper
% where the link is provided in the description above.
A = [1  delta_t; 
    0   1];

% Control Matix B represents the control signal input in the system. 
B = [(delta_t * delta_t)./2;    % see the details in the mentioned paper
    delta_t];

% The control input signal (Assume as a constant force for simplicity)
uk = 0.5 ;      % it is the input froce (f_t/m) in the mentioned paper

% The error matrix (or the confidence matrix): Pk states whether more weight
% should be given to the new measurement or to the model estimated value.
Pk = [1.5  0;
      0    1];          % Pk cannot be zero in initial guess (a posteriori)
Pk_prev = [];           % P_k_minus symbol in the paper (a Priori)

% Q is the process noise covariance. It represents the amount of uncertainty
% in the model. In practice, it is really difficult to know the exact
% value. Normally, it is assumed that the noise is Gaussian with zero mean.
Q = [0.0001  0 ;         % process noise sample value
     0        0.001];

 % The covariance of Q can be approximated as follows according to the example 
% described in wikipedia availabe at https://en.wikipedia.org/wiki/Kalman_filter
% Q = [(delta_t.^4)./4  (delta_t.^3)./2 ;     
%      (delta_t.^3)./2   (delta_t.^2)];

% H is the measurement matrix or Observation model matrix.  see the details
% in the paper provided in the description. 
H = [1  0;
     0  1];

% R is the measurement noise covariance. It represents the amount of errror
% in our measurement. In practice, this value can be found statistically.     
R = [0.5  0;        % measurement noise (feel free to play with the value)
     0    0.5];

% Buffers for plotting the results on the Graph
Xk_buffer = zeros(2, no_Samples);      % Kalman's estimated data buffer             
Z_buffer = zeros(2, no_Samples);       % noisy measurement data buffer
Pk_buffer = zeros(2, no_Samples);      % error covariance data buffer


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Kalman Iteration %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1 : no_Samples
    
    % Z is the measurement vector. Here, the simulated noise is added to the 
    % the real position and velocity data. In practice, the real data should come here.
    Z = Xtrue(:, i) + 10 .* randn;    % simulated noise multipy by xx times
    Z_buffer(:, i) = Z;
        
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Time Update (a.k.a. Prediction stage)%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    % 1. Project the state ahead
    Xk_prev = A * Xk + B * uk;          
    
    % 2. Project the error covariance ahead
    Pk_prev = A * Pk * A' + Q;  % Initial value for Pk shoud be guessed.                   
                                   
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Measurement Update (a.k.a. Correction or Innovation stage) %%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    S = H * Pk_prev * H' + R;   % prepare for the inverse Matrix
    
    % 1. compute the Kalman gain
    K = (Pk_prev * H')/S;       % K = Pk_prev * H' * inv(S); 
    
    % 2. update the estimate with measurement Zk
    Xk = Xk_prev + K * (Z - H * Xk_prev);
    Xk_buffer(:, i) = Xk;             % just for graphical result
    
    % 3. Update the error Covariance
    Pk = Pk_prev - K * H * Pk_prev;   % Pk = (I - K*H)Pk_prev
    
%     Pk_buffer(1, i) = Pk;           % for graphical result purpose only
    
end


% Moving Average Filter for comparing the results
movingAverage = zeros(2, no_Samples);
windowSize = 5;
b = (1/windowSize)*ones(1,windowSize);
a = 1;
movingAverage(1,:) = filter(b, a, Z_buffer(1,:));
movingAverage(2,:) = filter(b, a, Z_buffer(2,:));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot the resultant graph %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Graphcial comparison b/w Kalman filter and Noisy Measurement (Position)
figure;
% subplot(2,1,1)
plot(time_rate, Xtrue(1,:), 'b', 'LineWidth', 1.5);
hold on;
scatter(time_rate, Z_buffer(1,:), '+', 'k', 'LineWidth', 1.5);
plot(time_rate, Xk_buffer(1,:), 'm--*', 'LineWidth', 1);
plot(time_rate, movingAverage(1,:), 'r--d', 'LineWidth', 1);
title('Position of one-dimentional train tracking example');
xlabel('Time in seconds');
ylabel('Position of the Train (meter in 1D)');
legend('True value','Measurements','Kalman Filter', 'Moving Aveage');
hold off;

%Graphical result (Velocity)
figure
% subplot(2,1,2)
plot(time_rate, Xtrue(2,:), 'b', 'LineWidth', 1.5);
hold on;
scatter(time_rate, Z_buffer(2,:), '+', 'k', 'LineWidth', 1.5);
plot(time_rate, Xk_buffer(2,:), 'm--*', 'LineWidth', 1);
plot(time_rate, movingAverage(2,:), 'r--d', 'LineWidth', 1);
title('Velocity of one-dimentional train tracking example');
xlabel('Time in seconds');
ylabel('Velocity of the Train (m/s)');
legend('True value','Measurements','Kalman Filter', 'Moving Average');
hold off;


