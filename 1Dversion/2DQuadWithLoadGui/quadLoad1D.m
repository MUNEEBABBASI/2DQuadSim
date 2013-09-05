% 7/26/13
% quadLoad.m
% Simulation of quadrotor with cable-suspended load, assuming the cable remains taut
% Dependancies: calculateInputs1.m, calculateInputs2.m, calculateDerivatives.m, desiredTraj.m,
%   integrateMode1.m, integrateMode2.m, animateQuadLoad.m
% putvar.m (http://www.mathworks.com/matlabcentral/fileexchange/27106-putvar)



function quadLoad

clear all
close all
clc



    
    

%%%
% constants
g = 9.81; %m/s/s
mQ = 0.5; %mass of quadrotor, kg
mL = 0.08; %mass of load, kg
IQ = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3] ;
JQ = IQ(2,2) ;
l = 1; %length of cable, m

global s
s.g = g;
s.mQ = mQ;
s.mL = mL;
s.JQ = JQ;
s.l = l;

% control gains
% mode 1
kpx = [2*5*0.1; 20*5*0.1];%[2*5*0.1; 20*5*0.1];
kdx = [2*0.5; 10*2*0.5];%[2*0.5; 10*2*0.5];
kpL = 0.7;
kdL = 0.1;
kpQ = 140; %160;
kdQ = 56;%56;

% mode 2
kp = 0.8*10;
kd = 1*10;
kp_phi = 20; 
kd_phi = 7;



%%% 
% initial conditions 
tstart = 0;
tend = 5; %total time of simulation, s
[xT, dxT, d2xT, d3xT, d4xT, d5xT, d6xT] = desiredTraj(0, g, mQ, JQ);
[p_nom, dp_nom, d2p_nom, d3p_nom, d4p_nom, ...
    phiL_nom, dphiL_nom, d2phiL_nom, d3phiL_nom, d4phiL_nom, ...
    f_nom, phiQ_nom, dphiQ_nom, d2phiQ_nom] = calculateDerivatives(0, g, mL, mQ, JQ, l);

xL0 = [0 0]; %xT'; %initial position of load, m
vL0 = [0 0]; %dxT'; %initial velocity of quad, m/s
%xQ0 = [0 0]; 
%vQ0 = [0 0];
xQ0 = [0 xL0(1, 2)+l];
vQ0 = [0 vL0(1, 2)];
phiQ0 = 0; %phiQ_nom; % 0; %initial orientation of load, radians
phidotQ0 = 0; %dphiQ_nom; %0; %initial angular velocity of quad, radians/s

if pdist([xL0; xQ0]) == l
    phiL0 = 0; %phiL_nom; %atan2(xL0(1, 1)-xQ0(1, 1), xQ0(1, 2)-xL0(1, 2)); %initial orientation of load, radians
    phidotL0 = 0; %dphiL_nom; %0; %initial angular velocity of load, radians/s
    currentMode = 1; %mode 1 is when cable is taut
elseif pdist([xL0; xQ0]) < l
    phiL0 = 0;
    phidotL0 = 0;
    currentMode = 2; %mode 2 is when cable is slack
elseif pdist([xL0; xQ0]) > l
    disp('invalid initial positions!') %cable and quad can't be more than l apart
end

if currentMode == 1,
    % x1 = [xL vL phiL phidotL phiQ phidotQ]'
    x10 = [xL0 vL0 phiL0 phidotL0 phiQ0 phidotQ0];
elseif currentMode == 2,
    % x2 = [xL vL xQ vQ phiQ phidotQ]'
    x20 = [xL0 vL0 xQ0 vQ0 phiQ0 phidotQ0];
end




%%%

% make empty output vectors
tout = tstart;
if currentMode == 1,
    xout = [x10 0 0];
elseif currentMode == 2,
    xout = x20;
end
modeout = currentMode;
teout = [];
yeout = [];
ieout = [];
uout = zeros(1, 2); %record outputs, [f M]
desout = zeros(1, 7); %record desired states [xQ vQ phiQdes phiLdes]

% integrate
while tstart < tend && tout(length(tout))<tend,


    if currentMode == 1,

        
        options1 = odeset('Events', @slack);
        [t, x1, te, ye, ie] = integrateMode1([tstart tend], options1, x10, g, mL, mQ, JQ, l, kpx, kdx, kpL, kdL, kpQ, kdQ);
        

        % accumulate output
        nt = length(t);
        tout = [tout; t(2:nt)];
        modeout = [modeout; currentMode*ones(nt-1, 1)];
        xout = [xout; [x1(2:nt, :) zeros(nt-1, 2)]];
        
        utemp = zeros(nt, 2);
        destemp = zeros(nt, 7);
        for tempTime = 1:nt,
            [f, M, phiL_des, phiQ_des, d2phiQ_nom, p_des, dp_des] = calculateInputs1(t(tempTime), x1(tempTime, :)', g, mL, mQ, JQ, l, kpx, kdx, kpL, kdL, kpQ, kdQ);
            utemp(tempTime, :) = [f M];
            destemp(tempTime, :) = [x1(tempTime, 1)-l*p_des(1, 1) x1(tempTime, 2)-l*p_des(2, 1) ...
                x1(tempTime, 3)-l*dp_des(1, 1) x1(tempTime, 4)-l*dp_des(2, 1) ...
                phiQ_des phiL_des d2phiQ_nom ];
        end
        uout = [uout; utemp(2:nt, :)];
        desout = [desout; destemp(2:nt, :)];
        

        
        % only add the event if an event occured (te > 0)
        if te > 0,

            teout = [teout; te];
            yeout = [yeout; [ye zeros(1, 2)]];
            ieout = [ieout; ie];

        % set new initial conditions and flip mode       
        currentMode = 2;

        x20 = [x1(nt, 1) x1(nt, 2) x1(nt, 3) x1(nt, 4) ...
            x1(nt, 1)-l*sin(x1(nt, 5)) x1(nt, 2)+l*cos(x1(nt, 5)) ...
            x1(nt, 3)-l*x1(nt, 6)*cos(x1(nt, 5)) x1(nt, 4)-l*x1(nt, 6)*sin(x1(nt, 5)) ...
            x1(nt, 7) x1(nt, 8)];

        tstart = t(nt);
        
        end
        
    elseif currentMode == 2,
        
        options2 = odeset('Events', @taut);
        %options2 = odeset('Events', @collision2);
        [t, x2, te, ye, ie] = integrateMode2([tstart tend], options2, x20, g, mQ, JQ, kp, kd, kp_phi, kd_phi); 
        

        % accumulate output
        nt = length(t);
        tout = [tout; t(2:nt)];
        modeout = [modeout; currentMode*ones(nt-1, 1)];
        xout = [xout; x2(2:nt, :)];
        
        utemp = zeros(nt, 2);
        destemp = zeros(nt, 7);
        for tempTime = 1:nt,
            [f, M, phiQ_des, phiddotQ_des] = calculateInputs2(t(tempTime), x2(tempTime, :)', g, mQ, JQ, kp, kd, kp_phi, kd_phi);
            utemp(tempTime, :) = [f M];
            destemp(tempTime, :) = [0 0 ...
                0 0 ...
                phiQ_des 0 phiddotQ_des ];
        end
        uout = [uout; utemp(2:nt, :)];
        desout = [desout; destemp(2:nt, :)];
        

        

        % only add the event if an event occured (te > 0)
        if te > 0,

            
            teout = [teout; te];
            yeout = [yeout; ye];
            ieout = [ieout; ie];
            
                    
            % if the event wasn't a collision
            if (pdist([x2(nt, 1), x2(nt, 2); x2(nt, 5), x2(nt, 6)])>0)
                
                % set new intial conditions and flip mode
                currentMode = 1;
                x10 = [x2(nt, 1) x2(nt, 2) (mL*x2(nt, 3)+mQ*x2(nt, 7))/(mL+mQ) (mL*x2(nt, 4)+mQ*x2(nt, 8))/(mL+mQ) ...
                    real(acos(-(x2(nt, 2)-x2(nt, 6))/l)) 0 x2(nt, 9) x2(nt, 10)];
                tstart = t(nt);

                
            else
                
                % otherwise, a collison occured so end the program
                disp('quad and load collision!')
                break;
            end
        end
        


    end
end

% save output matrices
putvar(tout, modeout, xout, teout, yeout, ieout, uout, desout)





%%%
% construct output vectors

% construct quad position vector
totalTimeSteps = length(tout);
quadx = zeros(totalTimeSteps, 2);
for i = 1:totalTimeSteps,
    if (modeout(i, 1) == 1),
        quadx(i, :) = [xout(i, 1)-l*sin(xout(i, 5)) xout(i, 2)+l*cos(xout(i, 5))]; 
    elseif (modeout(i, 1) == 2),
        quadx(i, :) = [xout(i, 5) xout(i, 6)]; 
    end
end

%construct quad velocity vector
quadv = zeros(totalTimeSteps, 2);
for i = 1:totalTimeSteps,
    if (modeout(i, 1) == 1),
        quadv(i, :) = [xout(i, 3)-l*xout(i, 6)*cos(xout(i, 5)) xout(i, 4)-l*xout(i, 6)*sin(xout(i, 5))]; 
    elseif (modeout(i, 1) == 2),
        quadv(i, :) = [xout(i, 7) xout(i, 8)]; 
    end
end

%construct quad angle vector
quadphi = zeros(totalTimeSteps, 2);
for i = 1:totalTimeSteps,
    if (modeout(i, 1) == 1),
        quadphi(i, :) = [xout(i, 7) xout(i, 8)]; 
    elseif (modeout(i, 1) == 2),
        quadphi(i, :) = [xout(i, 9) xout(i, 10)]; 
    end
end

%construct load angle vector
loadphi = zeros(totalTimeSteps, 2);
for i = 1:totalTimeSteps,
    if (modeout(i, 1) == 1),
        loadphi(i, :) = [xout(i, 5) xout(i, 6)]; 
    elseif (modeout(i, 1) == 2),
        loadphi(i, :) = [0 0]; 
    end
end

%construct l - distance from quad to load
distDiff = zeros(totalTimeSteps, 1);
for i = 1:totalTimeSteps,
    distDiff(i, 1) = pdist([quadx(i, :); xout(i, 1:2)]);
end

% save quad output matricies
putvar(quadx, quadv, quadphi, loadphi)

% find desired trajectory
xTraj = zeros(length(tout), 2);
dxTraj = zeros(length(tout), 2);
d2xTraj = zeros(length(tout), 2);
d3xTraj = zeros(length(tout), 2);
d4xTraj = zeros(length(tout), 2);
d5xTraj = zeros(length(tout), 2);
d6xTraj = zeros(length(tout), 2);
for t = 1:length(tout),
    [xT, dxT, d2xT, d3xT, d4xT, d5xT, d6xT] = desiredTraj(tout(t), g, mQ, JQ, 1);
    xTraj(t, :) = xT';
    dxTraj(t, :) = dxT';
    d2xTraj(t, :) = d2xT';
    d3xTraj(t, :) = d3xT';
    d4xTraj(t, :) = d4xT';
    d5xTraj(t, :) = d5xT';
    d6xTraj(t, :) = d6xT';
    
    [xTQ, dxTQ, d2xTQ, d3xTQ, d4xTQ, d5xTQ, d6xTQ] = desiredTraj(tout(t), g, mQ, JQ, 2);
    xTrajQ(t, :) = xTQ';
    dxTrajQ(t, :) = dxTQ';
    d2xTrajQ(t, :) = d2xTQ';
    d3xTrajQ(t, :) = d3xTQ';
    d4xTrajQ(t, :) = d4xTQ';
    d5xTrajQ(t, :) = d5xTQ';
    d6xTrajQ(t, :) = d6xTQ';
end

% save tajectory properties
putvar(xTraj, dxTraj, d2xTraj, d3xTraj, d4xTraj, d5xTraj, d6xTraj)

%call animation
animateQuadLoad



%%%
% plot results


% plot position
figure()
hold on;
plot(xout(:, 1), xout(:, 2), 'b'); %x-z position of load
plot(quadx(:, 1), quadx(:, 2), 'k');
plot(xTraj(:, 1), xTraj(:, 2), 'r--');
legend('load', 'quadcopter', 'desired load trajectory');
xlabel('x position');
ylabel('z position');

% plot position over time
figure()
hold on;
plot(tout, quadx(:, 1)', 'b');
plot(tout, quadx(:, 2)', 'r');
plot(tout, xTrajQ(:, 1), 'b--');
plot(tout, xTrajQ(:, 2), 'r--');
xlabel('time (s)');
ylabel('position (m)');
title('quad position over time');
legend('x position', 'z position', 'desired x', 'desired z')

% plot velocity over time
figure()
hold on;
plot(tout, quadv(:, 1)', 'b');
plot(tout, quadv(:, 2)', 'r');
plot(tout, dxTrajQ(:, 1), 'b--');
plot(tout, dxTrajQ(:, 2), 'r--');
xlabel('time (s)');
ylabel('velocity (m/s)');
title('quad velocity over time');
legend('x velocity', 'z velocity', 'desired xdot', 'desired zdot');
% 
% % plot quad angle over time
% figure()
% hold on;
% plot(tout, quadphi(:, 1)'./pi*180, 'b');
% plot(tout, quadphi(:, 2)'./pi.*180, 'r');
% xlabel('time (s)');
% ylabel('angle (degrees)');
% title('quad angle over time');
% legend('phiQ', 'phiQdot');

% plot position over time
figure()
hold on;
plot(tout, xout(:, 1)', 'b');
plot(tout, xout(:, 2)', 'r');
plot(tout, xTraj(:, 1), 'b--');
plot(tout, xTraj(:, 2), 'r--');
xlabel('time (s)');
ylabel('position (m)');
title('load position over time');
legend('x position', 'z position', 'desired x', 'desired z')

% plot velocity over time
figure()
hold on;
plot(tout, xout(:, 3)', 'b');
plot(tout, xout(:, 4)', 'r');
plot(tout, dxTraj(:, 1), 'b--');
plot(tout, dxTraj(:, 2), 'r--');
xlabel('time (s)');
ylabel('velocity (m/s)');
title('load velocity over time');
legend('x velocity', 'z velocity', 'desired xdot', 'desired zdot');
% 
% % plot load angle over time (0 when rope is slack)
% figure()
% hold on;
% plot(tout, loadphi(:, 1)'./pi*180, 'b');
% plot(tout, loadphi(:, 2)'./pi*180, 'r');
% xlabel('time (s)');
% ylabel('angle (degrees)');
% title('load angle over time');
% legend('phiL', 'phiLdot');




% plot position over time
figure()
hold on;
plot(tout, quadx(:, 1)', 'b');
plot(tout, quadx(:, 2)', 'r');
plot(tout, xout(:, 1)', 'b--');
plot(tout, xout(:, 2)', 'r--');
xlabel('time (s)');
ylabel('position (m)');
title('position over time');
legend('quad x', 'quad z', 'load x', 'load z')

% plot velocity over time
figure()
hold on;
plot(tout, quadv(:, 1)', 'b');
plot(tout, quadv(:, 2)', 'r');
plot(tout, xout(:, 3)', 'b--');
plot(tout, xout(:, 4)', 'r--');
xlabel('time (s)');
ylabel('velocity (m/s)');
title('velocity over time');
legend('quad x', 'quad z', 'load x', 'load z')







% plot l over time
figure()
plot(tout, distDiff);
xlabel('time (s)');
ylabel('distance (m)');
title('distance between quad and load');


% plot inputs over time
figure()
plot(tout, uout(1:length(tout), 1));
xlabel('time (s)');
ylabel('f (N)');
title('force over time');

figure()
plot(tout, uout(1:length(tout), 2));
xlabel('time (s)');
ylabel('M');
title('moment over time');


% plot tension over time
figure()
T = zeros(length(d2xTraj(:, 1)), 1);
for i = 1:length(d2xTraj(:, 1))
    T(i, 1) = mL.* norm ( [d2xTraj(i, 1); d2xTraj(i, 2)] + [0; g]);
end
plot(tout, T);
xlabel('time (s)');
ylabel('tension (N)');
title('tension over time');






%%%%%
% event for when string goes from taut to slack
function [value, isterminal, direction] = slack(t, x1)

     
    [xT, dxT, d2xT, d3xT, d4xT, d5xT, d6xT] = desiredTraj(t, g, mQ, JQ, 1);

    value = mL * ( d2xT(2, 1) + g ); %when tension goes to 0, T = || mL (d/dt vL + ge3) ||
    isterminal = 1;
    direction = -1;
end



%%%%%
% event for when string goes from slack to taut
function [value, isterminal, direction] = taut(t, x2)

    
 value = [ ...
        x2(2, 1)- x2(6, 1); ... %when load and quad collide
        pdist([x2(1, 1), x2(2, 1); x2(5, 1), x2(6, 1)]) - l; ... %when string is at length l
        ];
    isterminal = [1; 1];
    direction = [0; 1];
end




end




