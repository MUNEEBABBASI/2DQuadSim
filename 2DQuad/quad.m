% 7/16/13
% quad.m
% Simulation of quadrotor 
% Dependancies: intergrateQuad.m, desiredTraj.m, findInputs.m
% putvar.m (http://www.mathworks.com/matlabcentral/fileexchange/27106-putvar)



function quad

clear all
close all
clc


%%%
% constants
g = 9.81; %m/s/s
mQ = 0.5; %mass of quadrotor, kg
JQ3D = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3];
JQ = JQ3D(2, 2); %moment of inertia of quad, kg*m*m

kp = 0.8;
kd = 1;
kp_phi = 20; 
kd_phi = 7;




%%% 
% initial conditions 
tstart = 0;
tend = 50; %total time of simulation, s
[xT, dxT, ~, ~, ~] = desiredTraj(0, g, mQ, JQ);
xQ0 = xT'; %initial position of quad, m
vQ0 = dxT'; %initial velocity of quad, m/s
phiQ0 = 0; %initial orientation of quad, radians
phidotQ0 = 0; %initial angular velocity of quad, radians/s





%%%

% set ic
x0 = [xQ0 vQ0 phiQ0 phidotQ0];

% integrate
[t, x] = integrateQuad([tstart tend], x0, g, mQ, JQ, kp, kd, kp_phi, kd_phi);
        
% save output matrices
xout = x;
tout = t;
putvar(tout, xout);




%%%
% plot results


% find desired trajectory
for t = 1:length(tout),
    [xT dxT d2xT d3xT d4xT] = desiredTraj(tout(t), g, mQ, JQ);
    xTraj(:, t) = xT; 
end


% plot position
figure()
hold on;
plot(xout(:, 1), xout(:, 2)); %x-z position 
plot(xTraj(1, :), xTraj(2, :), 'r--');
xlabel('y position');
ylabel('z position');
legend('actual traj', 'desired traj');


% plot position over time
figure()
hold on;
plot(tout, xout(:, 1)', 'b');
plot(tout, xout(:, 2)', 'r');
xlabel('time (s)');
ylabel('position (m)');
title('quad position over time');
legend('y position', 'z position')

% plot velocity over time
figure()
hold on;
plot(tout, xout(:, 3)', 'b');
plot(tout, xout(:, 4)', 'r');
xlabel('time (s)');
ylabel('velocity (m/s)');
title('quad velocity over time');
legend('y velocity', 'z velocity');

% plot angles over time
figure()
hold on;
plot(tout, xout(:, 5)', 'b');
plot(tout, xout(:, 6)', 'r');
xlabel('time (s)');
ylabel('value');
legend('angle (radians)', 'angular velocity (randians/s)');









end


