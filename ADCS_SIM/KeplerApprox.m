function [E] = KeplerApprox(M,e, maxError, maxIter)
%KEPPLERAPPROX Summary of this function goes here
%   Function to approximate a solution to Kepler's Equation, in order to
%   get eccentric Anomaly (E) from eccentricity (e) and mean anomaly (M)
    i = 0;
    if (e < 0.8)
        E = M;
    else
        E = pi;
    end
    F = E - e * sind(M) - M;
    while ((abs(F) > maxError) && (i < maxIter))
        E = E - F / (1 - e * cos(E));
        F = E - e * sin(E) - M;
        i = i + 1;
    end
end

