%% DATA
clear global
clear variables
clc

setup;
global dt simtime  
dt = 0.25; %should be multiple of 0.25 (I used 0.25s to sync with the sample frequency in the detumble algorithm)
simtime = 100;
    
 %% Functions
E2Q = @(x) [sin(x(1)/2)*cos(x(2)/2)*cos(x(3)/2) - cos(x(1)/2)*sin(x(2)/2)*sin(x(3)/2);
        cos(x(1)/2)*sin(x(2)/2)*cos(x(3)/2) + sin(x(1)/2)*cos(x(2)/2)*sin(x(3)/2);
        cos(x(1)/2)*cos(x(2)/2)*sin(x(3)/2) - sin(x(1)/2)*sin(x(2)/2)*cos(x(3)/2);
        cos(x(1)/2)*cos(x(2)/2)*cos(x(3)/2) + sin(x(1)/2)*sin(x(2)/2)*sin(x(3)/2)];

Q2E = @(q) [atan2(2*(q(4)*q(1)+q(2)*q(3)), 1-2*(q(1)^2+q(2)^2));
        asin(2*(q(4)*q(2)-q(3)*q(1)));
        atan2(2*(q(4)*q(3)+q(1)*q(2)), 1-2*(q(2)^2+q(3)^2))];  
    
%% Initial State
w0 = [0 -n_sat*100 0]';    
theta0 = deg2rad(30)*[1 1 1]';

x0_Euler = [theta0;w0];
x0_Msat = 0; %Mean Anomaly
x0_Epoch = datenum(2020,3,21,12,0,0);
x0_LLA = Orbit2LLA(x0_Msat,x0_Epoch);
[x0_Mag, ~, ~, ~, ~] = wrldmagm(x0_LLA(3), x0_LLA(1), x0_LLA(2), decyear(datevec(x0_Epoch)),'2020');

% Realign XYZ axis to LVLH frame, excluding effect of inclined orbit
M2F1 =   [0, 1, 0;
        -1, 0, 0;
         0, 0, 1];
x0_Mag = M2F1*x0_Mag;

%% Simulation
t=0:dt:simtime;
%initiating matrices
x_Euler = zeros(length(x0_Euler),length(t));
x_Euler(:,1) = x0_Euler;

x_Msat = zeros(1,length(t));
x_Msat(1) = x0_Msat;

x_LLA = zeros(length(x0_LLA),length(t));
x_LLA(:,1) = x0_LLA;

x_Epoch = zeros(1,length(t));
x_Epoch(1) = x0_Epoch;

x_Mag = zeros(length(x0_Mag),length(t));
x_Mag(:,1) = x0_Mag;

x_Mag_wgn1 = x_Mag;
x_Mag_wgn2 = x_Mag;

x_Ti = zeros(3,length(t));
x_Ti(:,1) = Ti;

x_tumble = zeros(3,length(t));
x_tumble(:,1) = [2, 2, 2]; %initial detumble parameter [2,2,2]

snr = -60; % signal-to-noise ratio in dB
%apply WHN to x0
x_Mag_wgn1(:,1) = awgn(x_Mag_wgn1(:,1), snr);
x_Mag_wgn2(:,1) = awgn(x_Mag_wgn2(:,1), snr);

%cntr is used when communication is lost with satellite while HIL testing
%(nan is return) to crop the plots to actual data
cntr = length(t);
for i = 2:length(t)   
    
    
    %TIMESTEP Simulation
    x_Euler(:,i) = multiRK4(@Diff_Euler,x_Euler(:,i-1),dt, dt);
    
    x_Msat(i) = x_Msat(i-1) + n_sat * dt;
    x_Epoch(i) = addtodate(x_Epoch(i-1),dt*1e3,'millisecond');
    
    % Eccentricity = 0, the Eccentric anomly = Mean anomaly
    %x_Esat = KeplerApprox(x_Msat(i)
    x_LLA(:,i) = Orbit2LLA(x_Msat(i),x_Epoch(i));
    
    [ XYZ_Mag , ~, ~, ~, ~] = wrldmagm(x_LLA(3,i), x_LLA(1,i), x_LLA(2,i), decyear(datevec(x_Epoch(i))),'2020');
    % Rotate the XYZ (North-East-Down) Magnetic field vector from it's frame to the
    % Body frame of the satellite. first to the Flight-Direction Frame
    % secondly to the Body Frame (*DCM(x_Euler(1:3,i)))
    
    % Rotation due to inclined orbit, amplitude of angle change = inclination
    % (angle = 0 at apo and perigee, angle = incl at AN&DN)
    angle = deg2rad(i_sat)*x_Msat(i);
    M2F2 =  [cos(angle), -sin(angle), 0;
            sin(angle),  cos(angle), 0;
                   0,              0,        1];
    
    % Rotation of body wrt LVLH
    F2B = DCM(x_Euler(1:3,i));

    % Implement rotations and assign values to the magnetometer
    XYZ_Mag = F2B*(M2F2*(M2F1*XYZ_Mag));
    x_Mag(:,i) = XYZ_Mag;
    
    % Adding white Gaussian noise
    x_Mag_wgn1(:,i) = awgn(XYZ_Mag, snr);
    x_Mag_wgn2(:,i) = awgn(XYZ_Mag, snr);
    
    
    teslaunit = 1e-9; %used to convert from nT to T
    
    % send sensordata in string form to python and get desired control torque back
    % generate commandstring to communicate with the python file
    % (make sure the path leads to the python file)
    commandStr = ['python "F:\Bart\Google Drive\Master\Q3\Microsat Engineering\DetumbleHILTesting\DetumbleAlgoHILTesting\DelfiPQ-PythonTestScripts\AlgoHILtest.py" ', ...
                num2str(x_Mag_wgn1(1,i)*teslaunit),' ', num2str(x_Mag_wgn1(2,i)*teslaunit),' ', num2str(x_Mag_wgn1(3,i)*teslaunit),' ', num2str(x_Mag_wgn2(1,i)*teslaunit),' ', num2str(x_Mag_wgn2(2,i)*teslaunit),' ', num2str(x_Mag_wgn2(3,i)*teslaunit)];
    % get desired dipole moment (which is a string)
    [status, str_M_Des] = system(commandStr);
    % crop to the data
    str_M_Des = str_M_Des(2:end-1);
    
    % split string to list of strings
    strnum = split(str_M_Des, ", ");

    % convert strings to doubles
    Mx = str2double(strnum(1));
    My = str2double(strnum(2));
    Mz = str2double(strnum(3));
    tumblex = str2double(strnum(4));
    tumbley = str2double(strnum(5));
    tumblez = str2double(strnum(6));
    
    % create vectors
    tumble = [tumblex, tumbley, tumblez]';
    Mi = [Mx, My, Mz]';
    
    % get torque from dipole moment
    Ti = cross(Mi, XYZ_Mag.*teslaunit);
    
    % sometimes connection can get lost while testing (bad internet or so),
    % then a nan is returned. whenever this happens, I want to break the
    % loop and plot the data op to this point.
    if isnan(Ti) | isnan(tumble)
        cntr = i -1;
        break
    end
    %append data to matrices
    x_Ti(:,i) = Ti;
    x_tumble(:,i) = tumble;
    
    
    % Print progress
    if mod(i,((length(t)-1)/100)) == 0
        fprintf('Progress %.0f%%\n',i/length(t)*100);
    end
    
end

%% Setting up plots;
close all;

figure(1);
subplot(4,2,1)
title('Euler - Angles of body','fontsize',14)
hold on;
grid on;
xlabel('Time [s]')
ylabel('Roll \theta_1 [Degrees]')
roll_plot = plot(t(1:cntr),rad2deg(x_Euler(1,1:cntr)),'Color','red');
subplot(4,2,3)
hold on;
grid on;
xlabel('Time [s]')
ylabel('Pitch \theta_2 [Degrees]')
pitch_plot = plot(t(1:cntr),rad2deg(x_Euler(2,1:cntr)),'Color','green');
subplot(4,2,5)
hold on;
grid on;
xlabel('Time [s]')
ylabel('Yaw \theta_3 [Degrees]')
yaw_plot = plot(t(1:cntr),rad2deg(x_Euler(3,1:cntr)),'Color','blue');

subplot(4,2,7)
hold on;
grid on;
title('Rotational Velocity of body','fontsize',14)
xlabel('Time [s]')
ylabel('Velocity [rad/s]')
w1_plot = plot(t(1:cntr),x_Euler(4,1:cntr),'Color','red');
w2_plot = plot(t(1:cntr),x_Euler(5,1:cntr),'Color','green');
w3_plot = plot(t(1:cntr),x_Euler(6,1:cntr),'Color','blue');
axis([0 cntr*dt -Inf Inf])
ylim([-pi pi]);
ylim auto


subplot(2,2,2)
hold on;
grid on;
title('GroundTrack','fontsize',14)
xlabel('Latitude [Deg]')
ylabel('Longitude [Deg]')
plot(x_LLA(2,1:cntr),x_LLA(1,1:cntr))


subplot(6,2,8)
hold on;
grid on;
xlabel('Time [s]')
ylabel('Mag X [nTesla]')
plot(t(1:cntr), x_Mag(1,1:cntr))
plot(t(1:cntr), x_Mag_wgn1(1,1:cntr))
plot(t(1:cntr), x_Mag_wgn2(1,1:cntr))

subplot(6,2,10)
hold on;
grid on;
xlabel('Time [s]')
ylabel('Mag Y [nTesla]')
plot(t(1:cntr),x_Mag(2,1:cntr))
plot(t(1:cntr),x_Mag_wgn1(2,1:cntr))
plot(t(1:cntr),x_Mag_wgn2(2,1:cntr))

subplot(6,2,12)
hold on;
grid on;
xlabel('Time [s]')
ylabel('Mag Z [nTesla]')
plot(t(1:cntr),x_Mag(3,1:cntr))
plot(t(1:cntr),x_Mag_wgn1(3,1:cntr))
plot(t(1:cntr),x_Mag_wgn2(3,1:cntr))

figure(2);
subplot(1,3,1)
hold on;
grid on;
xlabel('Time [s]')
ylabel('Control Torque X [Nm]')
plot(t(1:cntr),x_Ti(1,1:cntr))

subplot(1,3,2)
hold on;
grid on;
xlabel('Time [s]')
ylabel('Control Torque Y [Nm]')
plot(t(1:cntr),x_Ti(2,1:cntr))

subplot(1,3,3)
hold on;
grid on;
xlabel('Time [s]')
ylabel('Control Torque Z [Nm]')
plot(t(1:cntr),x_Ti(3,1:cntr))

figure(3);
subplot(1,3,1)
hold on;
grid on;
xlabel('Time [s]')
ylabel('tumble parameter X')
plot(t(1:cntr),x_tumble(1,1:cntr))

subplot(1,3,2)
hold on;
grid on;
xlabel('Time [s]')
ylabel('tumble parameter Y')
plot(t(1:cntr),x_tumble(2,1:cntr))

subplot(1,3,3)
hold on;
grid on;
xlabel('Time [s]')
ylabel('tumble parameter Z')
plot(t(1:cntr),x_tumble(3,1:cntr))