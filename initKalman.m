function [HMatrix, RMatrix, PMatrix] = initKalman()



    % kalman measurement/observation function H Matrix
    %                                      x     y     z    dx    dy    dz
                              HMatrix = [ 1.0   0.0   0.0   0.0   0.0   0.0 ; 
                                          0.0   1.0   0.0   0.0   0.0   0.0 ; 
                                          0.0   0.0   1.0   0.0   0.0   0.0 ];                                       
    % kalman measurement uncertainty
                              RMatrix =  eye(3)*5.0 ;
                                                                          
    % kalman initial uncertainty covariance matrix
    %                                       x     y     z    dx    dy    dz
                              PMatrix = [  0.8   0.0   0.0   0.4   0.0   0.0 ; 
                                           0.0   0.8   0.0   0.0   0.4   0.0 ; 
                                           0.0   0.0   0.8   0.0   0.0   0.4 ; 
                                           0.4   0.0   0.0   2.0   0.0   0.0 ; 
                                           0.0   0.4   0.0   0.0   2.0   0.0 ; 
                                           0.0   0.0   0.4   0.0   0.0   2.0 ] ;                                       
   end
                                       