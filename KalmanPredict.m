% @fileName KalmanPredict.m
% @author Melih Altun @2023

function [ k_x, k_P ] = KalmanPredict( k_x_in, k_P, dt)
    
    k_x = k_x_in;
    
    
    if nargin < 3
        dt = 1; %difference of current frame and last frame
    end

    k_F = [ 1.0   0.0   0.0    dt   0.0   0.0 ; 
            0.0   1.0   0.0   0.0    dt   0.0 ; 
            0.0   0.0   1.0   0.0   0.0    dt ; 
            0.0   0.0   0.0   1.0   0.0   0.0 ; 
            0.0   0.0   0.0   0.0   1.0   0.0 ; 
            0.0   0.0   0.0   0.0   0.0   1.0 ] ;

    f_Acceleration = 1.00 ;          % might need to be calculated
    f_sigmaAcceleration = 0.001 ;     % might need to be calculated
    
    k_w = [ 0.01*dt*dt/2*randn; 0.01*dt*dt/2*randn ; 0.01*dt*dt/2*randn ; 0 ; 0 ; 0 ] ;    
    %  process noise covariance                                   
    k_Q = [ (dt^4)/4    0.0      0.0   (dt^3)/2    0.0     0.0     ; 
              0.0    (dt^4)/4    0.0     0.0    (dt^3)/2   0.0     ; 
              0.0      0.0    (dt^4)/4   0.0      0.0    (dt^3)/2  ; 
            (dt^3)/2    0.0      0.0    dt^2      0.0       0.0    ; 
              0.0    (dt^3)/2    0.0     0.0     dt^2       0.0    ; 
              0.0      0.0    (dt^3)/2   0.0      0.0      dt^2    ];
    
    k_Q = f_sigmaAcceleration .* k_Q ;
    k_w = f_Acceleration .* k_w ;

    % prediction
    k_x = k_F * k_x + k_w ;
    k_P =  k_F * k_P * k_F' + k_Q ;
end
