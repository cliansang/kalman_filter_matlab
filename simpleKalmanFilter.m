%--------------------------------------------------------------------------
% Name:            simpleKalmanFilter.m
%
% Description:     Implementation of Kalman Fiter in Matlab according to 
%                  the paper entitled "An Introduction to the Kalman Filter"
%                  by Greg Welch and Gary Bishop. The paper is available at
%                  https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
%                  This example is based on the voltage reading error
%                  expressed in the above mentioned paper. The resultant
%                  graph could be a little bit different from the provided
%                  one in the paper because of the random generator used in
%                  this specific implementation.
%
% Author:          Cung Lian Sang
%                  salaikyonekyone12@gmail.com (or)
%                  csang@tecahfak.uni-bielefeld.de
%
% Date:            August 10, 2018
%--------------------------------------------------------------------------


close all; clear; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Initial Parameters %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Nsamples = 50;
Vtrue = -0.37727;           % true value
sigma_meas = 0.1;           % measurement sigma value (error variation)
iteration = 1: 50;
Xtrue = zeros(1, Nsamples); % Vector of true value (for graph)
Xtrue(:) = Vtrue;



% Previous state estimated value
Xk_prev = [];

% Current state estimate (initial guess): it is guessed that 0.2 times error
% of the true value in initial state
Xk =  Vtrue * 0.2;       % this is our initial guess

% Matrix A represents the dynamics of the system: depending on the system.
% Here we read voltage data from only one reading. so A = 1 (one value matrix)
A = 1;

% The error matrix (or the confidence matrix): Pk states whether more weight
% should be given to the new measurement or to the model estimated value.
Pk = 1.0;               % Pk cannot be zero in initial guess.

% Q is the process noise covariance. It represents the amount of uncertainty
% in the model. In practice, it is really difficult to know the exact
% value. Normally, it is assumed that the noise is Gaussian with zero mean.
Q = 1 * 10^-5 ;         % process noise

% H is the measurement matrix. Again only one reading. So a matix of 1.
% H = [1 0];
H = 1;

% R is the measurement noise covariance. It represents the amount of errror
% in our measurement. Here, a matrix with single value again. In practice,
% this value can be found statistically before the KF is applied.
R = 0.1 * 0.1 ;        % measurement noise (feel free to play with the value)


% Buffers for plotting the results on the Graph
Xk_buffer = zeros(1,Nsamples);      % Kalman's estimated data buffer
Xk_buffer(:) = Xk;                
Z_buffer = zeros(1,Nsamples);       % noisy measurement data buffer
Pk_buffer = zeros(1, Nsamples);     % error covariance data buffer


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Kalman Iteration %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1 : Nsamples
    
    % Z is the measurement vector. Here, the simulated noise is added to the 
    % voltage reading data. In practice, the real data should come here.
    Z = Xtrue(i)+ sigma_meas * randn;    % simulated noise multipy by 0.1 variation
    Z_buffer(i) = Z;
        
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Time Update (a.k.a. Predict stage)%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    % 1. Project the state ahead
    Xk_prev = A * Xk ;          % There is no control signal. So, uk = 0;
    
    % 2. Project the error covariance ahead
    Pk_prev = A * Pk * A' + Q;  % Initial value for Pk shoud be guessed.                   
                                   
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Measurement Update (a.k.a. Correct or Innovation stage) %%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    S = H * Pk_prev * H' + R;   % prepare for the inverse
    
    % 1. compute the Kalman gain
    K = Pk_prev * H' * inv(S); 
    
    % 2. update the estimate with measurement Zk
    Xk = Xk_prev + K * (Z - H * Xk_prev);
    Xk_buffer(:, i) = Xk;
    
    % 3. Update the error Covariance
    Pk = Pk_prev - K * H * Pk_prev;   % Pk = (I - K*H)Pk_prev
    
    Pk_buffer(i) = Pk;        % for graphical result purpose only
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Plot the resultant graph %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Graphcial comparison b/w Kalman filter and Noisy Measurement
figure;
plot(iteration, Xtrue,'b', 'LineWidth', 1.5);
hold on;
scatter(iteration, Z_buffer, '+', 'k', 'LineWidth', 1.5);
plot(iteration, Xk_buffer(1,:),'m--*', 'LineWidth', 1);
title('Kalman Filter Simulation for Votage Reading example');
xlabel('Iteration');
ylabel('Voltage (V)');
legend('True value','Measurements','Kalman Filter');
hold off;

% Graph of Error Covariance 
figure
plot(iteration, Pk_buffer,'b', 'Linewidth', 1.5);
title('Error Covariance (P_k) upon Iterations');
xlabel('Iteration');
ylabel('Voltage^2');

