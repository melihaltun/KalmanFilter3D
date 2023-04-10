function [ k_x, k_P ] = Kalman( k_z, k_x_in, k_P, k_R, k_H, dt)
    
    k_x = k_x_in;
    k_I = 1.0*eye(6);
    
    if nargin < 6
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
  
    % prediction, motion update
    k_x = k_F * k_x + k_w ;
    k_P =  k_F * k_P * k_F' + k_Q ;   
    
    %Error & Gain
    k_y = double ( k_z - k_H * k_x );
   %k_y = double ( k_z - k_x );
    k_S = double (( k_H * k_P * k_H' ) + k_R  );
    k_K = double ( ( k_P * k_H' ) * k_S^-1 ) ;
    
    % measurement update
    k_x = k_x + ( k_K * k_y ) ; 
    k_P = ( k_I - k_K * k_H ) * k_P ;
end
