% @fileName KalmanUpdate.m
% @author Melih Altun @2023

function [ k_x, k_P ] = KalmanUpdate( k_z, k_x_in, k_P, k_R, k_H)
    
    k_x = k_x_in;
    k_I = 1.0*eye(6);
    
    %Error & Gain
    k_y = double ( k_z - k_H * k_x );
    k_S = double (( k_H * k_P * k_H' ) + k_R  );
    k_K = double ( ( k_P * k_H' ) * k_S^-1 ) ;
    
    % measurement update
    k_x = k_x + ( k_K * k_y ) ; 
    k_P = ( k_I - k_K * k_H ) * k_P ;
end
