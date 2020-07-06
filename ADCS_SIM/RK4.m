function [ x_next ] = RK4(Diff, x , h)
%RK4 Summary of this function goes here
%   Detailed explanation goes here [theta ; Omega]
    k1 = Diff(x);
    k2 = Diff(x+(h/2)*k1);
    k3 = Diff(x+(h/2)*k2);
    k4 = Diff(x+h*k3);
    
    x_next = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);

end

