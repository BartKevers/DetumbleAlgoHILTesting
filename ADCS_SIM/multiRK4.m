function [ x_next ] = multiRK4(Diff, x_in , h, dt)
%RK4 Summary of this function goes here
%   Detailed explanation goes here [theta ; Omega]
x_next = x_in;
for i=1:(dt/h)
    x_next = RK4(Diff,x_next,h);
end
end

