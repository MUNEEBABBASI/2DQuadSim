% 9/5/13
% quadLoad.m
% Simulation of quadrotor with cable-suspended load
% 1D version
% no controller running
% Dependancies: calculateDerivatives.m, desiredTraj.m,
%   integrateMode1.m, integrateMode2.m, animateQuadLoad.m
% putvar.m (http://www.mathworks.com/matlabcentral/fileexchange/27106-putvar)



function quadLoad1D

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




%%% 
% initial conditions 
tstart = 0;
tend = 1; %total time of simulation, s
[xT, dxT, d2xT] = desiredTraj(0, g, mQ, JQ);




xL0 = xT; %initial position of load, m
vL0 = dxT; %dxT'; %initial velocity of quad, m/s
xQ0 = xL0+l;
vQ0 = vL0;



if xQ0-xL0 == l
    currentMode = 1; %mode 1 is when cable is taut
elseif xQ0-xL0 < l
    currentMode = 2; %mode 2 is when cable is slack
elseif xQ0-xL0 > l
    disp('invalid initial positions!') %cable and quad can't be more than l apart
end

if currentMode == 1,
    % x1 = [xL vL]'
    x10 = [xL0 vL0];
elseif currentMode == 2,
    % x2 = [xL vL xQ vQ]'
    x20 = [xL0 vL0 xQ0 vQ0];
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
uout = zeros(1, 1); %record outputs, [f M]

% integrate
while tstart < tend && tout(length(tout))<tend,


    if currentMode == 1,

        
        options1 = odeset('Events', @slack);
        [t, x1, te, ye, ie] = integrateMode1([tstart tend], options1, x10, g, mL, mQ, JQ, l);
        

        % accumulate output
        nt = length(t);
        tout = [tout; t(2:nt)];
        modeout = [modeout; currentMode*ones(nt-1, 1)];
        xout = [xout; [x1(2:nt, :) zeros(nt-1, 2)]];
        
        utemp = zeros(nt, 1);
        for tempTime = 1:nt,
            [xT, dxT, d2xT] = desiredTraj(tout(tempTime), g, mQ, JQ, 1); 
            utemp(tempTime, 1) = (mL+mQ)*(d2xT+g);
        end
        uout = [uout; utemp(2:nt, 1)];
        

        
        % only add the event if an event occured (te > 0)
        if te > 0,

            teout = [teout; te];
            yeout = [yeout; [ye zeros(1, 2)]];
            ieout = [ieout; ie];

        % set new initial conditions and flip mode       
        currentMode = 2;

        % x2 = [xL vL xQ vQ]'
        x20 = [x1(nt, 1) x1(nt, 2) x1(nt, 1)+l x1(nt, 2)];
        tstart = t(nt);
        
        end
        
    elseif currentMode == 2,
        
        options2 = odeset('Events', @taut);
        %options2 = odeset('Events', @collision2);
        [t, x2, te, ye, ie] = integrateMode2([tstart tend], options2, x20, g, mQ, JQ); 
        

        % accumulate output
        nt = length(t);
        tout = [tout; t(2:nt)];
        modeout = [modeout; currentMode*ones(nt-1, 1)];
        xout = [xout; x2(2:nt, :)];
        
        utemp = zeros(nt, 1);
        for tempTime = 1:nt,
            [xT, dxT, d2xT] = desiredTraj(tout(tempTime), g, mQ, JQ, 2); 
            utemp(tempTime, 1) = mQ*(d2xT+g);
        end
        uout = [uout; utemp(2:nt, 1)];
        

        

        % only add the event if an event occured (te > 0)
        if te > 0,

            
            teout = [teout; te];
            yeout = [yeout; ye];
            ieout = [ieout; ie];
            
                    
            % if the event wasn't a collision
            if ((x2(nt, 3)-x2(nt, 1))>0)
                
                % set new intial conditions and flip mode
                currentMode = 1;
                x10 = [x2(nt, 1) x2(nt, 4)];
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
putvar(tout, modeout, xout, teout, yeout, ieout, uout)




%%%
% construct output vectors

% construct quad position vector
totalTimeSteps = length(tout);
quadx = zeros(totalTimeSteps, 1);
for i = 1:totalTimeSteps,
    if (modeout(i, 1) == 1),
        quadx(i, :) = xout(i, 1)+l; 
    elseif (modeout(i, 1) == 2),
        quadx(i, :) = xout(i, 3); 
    end
end

%construct quad velocity vector
quadv = zeros(totalTimeSteps, 1);
for i = 1:totalTimeSteps,
    if (modeout(i, 1) == 1),
        quadv(i, :) = xout(i, 2); 
    elseif (modeout(i, 1) == 2),
        quadv(i, :) = xout(i, 4); 
    end
end



%construct l - distance from quad to load
distDiff = zeros(totalTimeSteps, 1);
for i = 1:totalTimeSteps,
    distDiff(i, 1) = quadx(i, :) - xout(i, 1);
end

% save quad output matricies
putvar(quadx, quadv)

% find desired trajectory
xTraj = zeros(length(tout), 2);
dxTraj = zeros(length(tout), 2);
d2xTraj = zeros(length(tout), 2);

xTrajQ = zeros(length(tout), 2);
dxTrajQ = zeros(length(tout), 2);
d2xTrajQ = zeros(length(tout), 2);
for t = 1:length(tout),
    [xT, dxT, d2xT] = desiredTraj(tout(t), g, mQ, JQ, 1);
    xTraj(t, :) = xT';
    dxTraj(t, :) = dxT';
    d2xTraj(t, :) = d2xT';
    
    [xTQ, dxTQ, d2xTQ] = desiredTraj(tout(t), g, mQ, JQ, 2);
    xTrajQ(t, :) = xTQ';
    dxTrajQ(t, :) = dxTQ';
    d2xTrajQ(t, :) = d2xTQ';

end

% save tajectory properties
putvar(xTraj, dxTraj, d2xTraj)

%call animation
animateQuadLoad



%%%
% plot results



% plot position over time
figure()
hold on;
plot(tout, quadx(:, 1)', 'r--');
plot(tout, xTrajQ(:, 1), 'r');
xlabel('time (s)');
ylabel('position (m)');
title('quad position over time');
legend('x position', 'desired x')

% plot velocity over time
figure()
hold on;
plot(tout, quadv(:, 1)', 'b');
plot(tout, dxTrajQ(:, 1), 'b--');
xlabel('time (s)');
ylabel('velocity (m/s)');
title('quad velocity over time');
legend('x velocity', 'desired xdot');


% plot position over time
figure()
hold on;
plot(tout, xout(:, 1)', 'b');
plot(tout, xTraj(:, 1), 'b--');
xlabel('time (s)');
ylabel('position (m)');
title('load position over time');
legend('x position', 'desired x')

% plot velocity over time
figure()
hold on;
plot(tout, xout(:, 2)', 'b');
plot(tout, dxTraj(:, 1), 'b--');
xlabel('time (s)');
ylabel('velocity (m/s)');
title('load velocity over time');
legend('x velocity', 'desired xdot');





% plot position over time
figure()
hold on;
plot(tout, quadx(:, 1)', 'b');
plot(tout, xout(:, 1)', 'r');
xlabel('time (s)');
ylabel('position (m)');
title('position over time');
legend('quad x','load x')

% plot velocity over time
figure()
hold on;
plot(tout, quadv(:, 1)', 'b');
plot(tout, xout(:, 2)', 'r');
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



% plot tension over time
figure()
T = zeros(length(d2xTraj(:, 1)), 1);
for i = 1:length(d2xTraj(:, 1))
    T(i, 1) = mL.* (d2xTraj(i, 1)+g);
end
plot(tout, T);
xlabel('time (s)');
ylabel('tension (N)');
title('tension over time');






%%%%%
% event for when string goes from taut to slack
function [value, isterminal, direction] = slack(t, x1)

     
    [xT, dxT, d2xT] = desiredTraj(t, g, mQ, JQ, 1);

    value = mL * ( d2xT(1, 1) + g ); %when tension goes to 0, T = || mL (d/dt vL + ge3) ||
    isterminal = 1;
    direction = -1;
end



%%%%%
% event for when string goes from slack to taut
function [value, isterminal, direction] = taut(t, x2)

    
 value = [ ...
        x2(3, 1)- x2(1, 1); ... %when load and quad collide
        x2(3, 1)- x2(1, 1) - l; ... %when string is at length l
        ];
    isterminal = [1; 1];
    direction = [0; 1];
end




end




