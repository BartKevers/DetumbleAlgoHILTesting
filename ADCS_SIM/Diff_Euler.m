function [ xdot ] = Diff_Euler(x)
    global n_sat J Td Ti skew
    theta = x(1:3);
    w = x(4:6);
    C = DCM(theta);
    
    % wAN_dot
    wdot = J\(Td  + Ti + 3*n_sat^2*skew(C(:,3))*J*C(:,3) - skew(w)*J* w);
    

    %% kinematics of rotation due to angular momentum and wBN:

    A = [1 0 -sin(theta(2));
        0 cos(theta(1)) sin(theta(1))*cos(theta(2));
        0 -sin(theta(1)) cos(theta(1))*cos(theta(2))];
        
    thetaDot = A\(w + n_sat*C(:,2));
    
    %% combine
    xdot = zeros(size(x));
    xdot(1:3) = thetaDot;
    xdot(4:6) = wdot;
    
end

