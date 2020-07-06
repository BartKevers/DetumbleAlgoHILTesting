function [ C ] = DCM( x )
%DCM_EULER Summary of this function goes here
%   Detailed explanation goes here
global skew
if(length(x) == 3)
    %Euler 
    C = [cos(x(2))*cos(x(3)), cos(x(2))*sin(x(3)), -sin(x(2));
    sin(x(1))*sin(x(2))*cos(x(3))-cos(x(1))*sin(x(3)), sin(x(1))*sin(x(2))*sin(x(3))+cos(x(1))*cos(x(3)), sin(x(1))*cos(x(2));
    cos(x(1))*sin(x(2))*cos(x(3))+sin(x(1))*sin(x(3)), cos(x(1))*sin(x(2))*sin(x(3))-sin(x(1))*cos(x(3)), cos(x(1))*cos(x(2))];
end
if (length(x) == 4)
    %quaternions
    q = x(1:3);
    Q = skew(q);
    q4 = x(4);
    C = (q4^2 - (transpose(q)*q))*eye(3)+2*(q*transpose(q)) - 2*q4*Q;
end

