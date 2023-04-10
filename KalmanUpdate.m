% @fileName KalmanUpdate.m
% @author Melih Altun @2023

function [x, P] = KalmanUpdate( z, x_inp, P, R, H)
    
    x = x_inp;
    I = 1.0*eye(6);
    
    %Error & Gain
    y = z - H * x;
    S = ( H * P * H' ) + R;
    K = ( P * H' ) * S^-1;
    
    % measurement update
    x = x + ( K * y ) ; 
    P = ( I - K * H ) * P ;
end
