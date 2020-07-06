global skew n_sat a_sat e_sat Omega_sat omega_sat i_sat J Td Ti KpEU KdEU
%% Functions  
skew = @(in) [0 -in(3) in(2);
            in(3) 0 -in(1);
            -in(2) in(1) 0];
        
%% PARAMETERS

%orbital parameters
h_sat = 500e3;   %Satellite Height
R_Earth = 6371e3; %earth raduis
mu_Earth = 3.986004418e14; %earth gravitational parameter

a_sat = (R_Earth+h_sat); %semi major axis
e_sat = 0; %orbit eccentricity
i_sat = 30; %inclination in degrees
omega_sat = deg2rad(40); %argument of periapsis
Omega_sat = deg2rad(340); %longitude of a.n.
M_sat = deg2rad(80); %Mean anomoly in degrees
n_sat = sqrt(mu_Earth/((a_sat)^3) ); %mean motion radians per second;

period_sat = 2*pi*sqrt(a_sat^3 / mu_Earth);


%satellite parameters
J = diag([1.731 1.726 0.264])*1e-3; %diag([124.531 124.586 0.704]); %inertial tensor
Td = [0,0,0]'; %[0.0001 0.0001 0.0001]';% disturbance torques


%% CONTROL PARAMETERS
Ti = [0.000 0.000 0.000]';

KpEU = Td/deg2rad(0.1) - [4*n_sat^2*(J(2,2)-J(3,3)); 3*n_sat^2*(J(1,1)-J(3,3)); n_sat^2*(J(2,2)-J(1,1))]; 
KdEU = sqrt(2*KpEU.*diag(J));