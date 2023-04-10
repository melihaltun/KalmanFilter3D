% @fileName KalmanInit.m
% @author Melih Altun @2023

function [Hmatrix, RMatrix, PMatrix] = KalmanInit()
                           
    % kalman measurement uncertainty
    RMatrix =  eye(3)*0.5;

% kalman measurement/observation function H Matrix
%                x     y     z    dx    dy    dz
    Hmatrix = [ 1.0   0.0   0.0   0.0   0.0   0.0 ; 
                0.0   1.0   0.0   0.0   0.0   0.0 ; 
                0.0   0.0   1.0   0.0   0.0   0.0 ];    
                                                                          
% kalman initial uncertainty covariance matrix: assumes initial velocities are unknown
%                x     y     z    dx    dy    dz
   PMatrix = [  0.8   0.0   0.0   0.4   0.0   0.0 ; 
                0.0   0.8   0.0   0.0   0.4   0.0 ; 
                0.0   0.0   0.8   0.0   0.0   0.4 ; 
                0.4   0.0   0.0   2.5   0.0   0.0 ; 
                0.0   0.4   0.0   0.0   2.5   0.0 ; 
                0.0   0.0   0.4   0.0   0.0   2.5 ];                                       
   end
                                       