function [r_lla] = Keppler2LLA(E_sat, epoch_sat)
    %KEPPLER2LATLON Summary of this function goes here
    %   Detailed explanation goes here
    %coordinate in orbital frame (https://en.wikipedia.org/wiki/Kepler_orbit)
    global a_sat e_sat Omega_sat omega_sat i_sat
    x_orbit = a_sat * (cos(E_sat) - e_sat);  %towards periapsis (semi-major axis)
    y_orbit = a_sat*sqrt(1-e_sat^2) * sin(E_sat);  %towards (minor axis b)
    z_orbit = 0; %towards inertial vector h

    Rz_Omega = [ ... %longitude of a.n.
        [cos(Omega_sat) sin(Omega_sat) 0]; ...
        [-sin(Omega_sat) cos(Omega_sat) 0]; ...
        [0 0 1]];
    Rx_i = [ ...  %inclination
        [1 0 0]; ...
        [0 cos(i_sat) sin(i_sat)]; ...
        [0 -sin(i_sat) cos(i_sat)]];
    Rz_omega = [ ... %argument of periapsis
        [cos(omega_sat) sin(omega_sat) 0]; ...
        [-sin(omega_sat) cos(omega_sat) 0]; ...
        [0 0 1]];

    % Rotate Orbital frame coordinates 'back' to ECI frame
    r_ECI = inv(Rz_Omega)*inv(Rx_i)*inv(Rz_omega)*[x_orbit, y_orbit, z_orbit]';
    R_Earth = 6371e3;
    test = norm(r_ECI) - R_Earth; %this should be h_sat? but it is not.

    r_lla = eci2lla(r_ECI',datevec(epoch_sat));
end

