% @fileName KalmanPredict.m
% @author Melih Altun @2023

function [x, P] = KalmanPredict(x_inp, P, dt)
    
    x = x_inp;    
    
    if nargin < 3
        dt = 1; %difference of current frame and last frame
    end

    % state transition / forcing matrix
    F  =  [ 1.0   0.0   0.0    dt   0.0   0.0 ; 
            0.0   1.0   0.0   0.0    dt   0.0 ; 
            0.0   0.0   1.0   0.0   0.0    dt ; 
            0.0   0.0   0.0   1.0   0.0   0.0 ; 
            0.0   0.0   0.0   0.0   1.0   0.0 ; 
            0.0   0.0   0.0   0.0   0.0   1.0 ] ;

    f_Acceleration = 0.01 ;          % might need to be calculated
    f_sigmaAcceleration = 0.07 ;     % might need to be calculated
    
    % Process noise
    w = [ dt*dt/2*randn; dt*dt/2*randn ; dt*dt/2*randn ; 0 ; 0 ; 0 ] ;    
    
    %  process noise covariance                                   
    Q = [ (dt^4)/4    0.0      0.0   (dt^3)/2    0.0     0.0     ; 
              0.0    (dt^4)/4    0.0     0.0    (dt^3)/2   0.0     ; 
              0.0      0.0    (dt^4)/4   0.0      0.0    (dt^3)/2  ; 
            (dt^3)/2    0.0      0.0    dt^2      0.0       0.0    ; 
              0.0    (dt^3)/2    0.0     0.0     dt^2       0.0    ; 
              0.0      0.0    (dt^3)/2   0.0      0.0      dt^2    ];
    
    Q = f_sigmaAcceleration^2 .* Q ;
    w = f_Acceleration^2 .* w ;

    % prediction
    x = F * x + w ;
    P =  F * P * F' + Q ;
end
