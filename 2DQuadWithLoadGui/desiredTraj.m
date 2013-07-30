% 7/26/13
% desiredTraj.m
% desired quadrotor trajectory input
% Dependencies: optTraj.m
%
% inputs: 
%   t: real value, current time
%   g, mQ, JQ: real values, constants
% outputs:
%   xT, dxT, d2xT, d3xT, d4xT, d5xT, d6xT: each a 2x1 vector, position and higher derivatives of trajectory at current time

%%%%%
% Specify the position and derivatives of the desired trajectory
function [xT, dxT, d2xT, d3xT, d4xT, d5xT, d6xT] = desiredTraj(t, g, mQ, JQ)

% specify trajectory

%     T=5;
%     th_max = 2*pi/180 ;
%     L = 1 ;
%     Gx = L*sin(th_max) ; Gy = 0*-(L-L*cos(th_max)) ;
%     xT = [0 + Gx*sin(2*pi*1/T*t) ; -Gy-L + Gy*cos(2*pi*1/T*t)] ;
%     vT = (2*pi*1/T)*[Gx*cos(2*pi*1/T*t) ; Gy*-sin(2*pi*1/T*t)] ;
%     aT = (2*pi*1/T)^2*[Gx*-sin(2*pi*1/T*t) ; Gy*-cos(2*pi*1/T*t)] ;
%     jT = (2*pi*1/T)^3*[Gx*-cos(2*pi*1/T*t) ; Gy*sin(2*pi*1/T*t)] ;
%     sT = (2*pi*1/T)^4*[Gx*sin(2*pi*1/T*t) ; Gy*cos(2*pi*1/T*t)] ;


%     xT = [ 5*cos(t); 5*sin(t)];
%     dxT = [ -5*sin(t) ; 5*cos(t) ];
%     d2xT = [ -5*cos(t) ; -5*sin(t) ];
%     d3xT = [ 5*sin(t) ; -5*cos(t)];
%     d4xT = [ 5*cos(t) ; 5*sin(t)];
%     d5xT = [-5*sin(t) ; 5*cos(t)];
%     d6xT = [-5*cos(t) ; -5*sin(t)];

% xT = [t; t];
% dxT = [1; 1];
% d2xT = [0; 0];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];
% 
% xT = [t; 0];
% dxT = [1; 0];
% d2xT = [0; 0];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];

% xT = [0; t];
% dxT = [0; 1];
% d2xT = [0; 0];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];

% 
% xT = [t^2; t];
% dxT = [2*t; 1];
% d2xT = [2; 0];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];

% xT = [t; t^2];
% dxT = [1; 2*t];
% d2xT = [0; 2];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];

% xT = [t; t^3];
% dxT = [1; 3*t];
% d2xT = [0; 3];
% d3xT = [0; 0];
% d4xT = [0; 0];
% d5xT = [0; 0];
% d6xT = [0; 0];



%%%
% design an optimal trajectory
% based on snapwaypoint.m, written by Daniel Mellinger

persistent s

% waypoints
numWaypoints = 4;
posT = [0; 2; 4; 6];
posY = [0; 1; 1; 0];
posZ = [0; 0; 2; 2];


% optimize a trajectory between these waypoints

if(isempty(s))

    for i = 1:numWaypoints-1,
        s.y{i} = optTraj([posY(i, 1),0,0,0,0,0,0,0]',[posY(i+1, 1),0,0,0,0,0,0,0]');
        s.z{i} = optTraj([posZ(i, 1),0,0,0,0,0,0,0]',[posZ(i+1, 1),0,0,0,0,0,0,0]');
        
        s.dy{i} = polyder(s.y{i});
        s.d2y{i} = polyder(s.dy{i});
        s.d3y{i} = polyder(s.d2y{i});
        s.d4y{i} = polyder(s.d3y{i});
        s.d5y{i} = polyder(s.d4y{i});
        s.d6y{i} = polyder(s.d5y{i});
        
        s.dz{i} = polyder(s.z{i});
        s.d2z{i} = polyder(s.dz{i});
        s.d3z{i} = polyder(s.d2z{i});
        s.d4z{i} = polyder(s.d3z{i});
        s.d5z{i} = polyder(s.d4z{i});
        s.d6z{i} = polyder(s.d5z{i});

    end

    
end

if(t<posT(1, 1))
    xT = [posY(1, 1); posZ(1, 1)];
    dxT = [ 0;0 ];
    d2xT = [ 0;0 ];
    d3xT = [ 0;0 ];
    d4xT = [ 0;0 ];
    d5xT = [ 0;0 ];
    d6xT = [ 0;0 ];
    
elseif(t<(posT(numWaypoints, 1)))
    for i = 1:numWaypoints-1,
        if (t < posT(i+1, 1))
            scaledt = (t-posT(i, 1))/(posT(i+1, 1)-posT(i, 1));
            xT = [polyval(s.y{i},scaledt);polyval(s.z{i},scaledt)];
            dxT = 1/(posT(i+1, 1)-posT(i, 1))*[polyval(s.dy{i},scaledt);polyval(s.dz{i},scaledt)];
            d2xT = 1/(posT(i+1, 1)-posT(i, 1))^2*[polyval(s.d2y{i},scaledt);polyval(s.d2z{i},scaledt)];
            d3xT = 1/(posT(i+1, 1)-posT(i, 1))^3*[polyval(s.d3y{i},scaledt);polyval(s.d3z{i},scaledt)];
            d4xT = 1/(posT(i+1, 1)-posT(i, 1))^4*[polyval(s.d4y{i},scaledt);polyval(s.d4z{i},scaledt)];
            d5xT = 1/(posT(i+1, 1)-posT(i, 1))^5*[polyval(s.d5y{i},scaledt);polyval(s.d5z{i},scaledt)];
            d6xT = 1/(posT(i+1, 1)-posT(i, 1))^6*[polyval(s.d6y{i},scaledt);polyval(s.d6z{i},scaledt)];

            return
        end
    end

%     scaledt = (t-t0)/(t1-t0);
%     xT = [polyval(s.y,scaledt);polyval(s.z,scaledt)];
%     dxT = 1/(t1-t0)*[polyval(s.dy,scaledt);polyval(s.dz,scaledt)];
%     d2xT = 1/(t1-t0)^2*[polyval(s.d2y,scaledt);polyval(s.d2z,scaledt)];
%     d3xT = 1/(t1-t0)^3*[polyval(s.d3y,scaledt);polyval(s.d3z,scaledt)];
%     d4xT = 1/(t1-t0)^4*[polyval(s.d4y,scaledt);polyval(s.d4z,scaledt)];
%     d5xT = 1/(t1-t0)^5*[polyval(s.d5y,scaledt);polyval(s.d5z,scaledt)];
%     d6xT = 1/(t1-t0)^6*[polyval(s.d6y,scaledt);polyval(s.d6z,scaledt)];
% elseif (t<t2)
%     scaledt = (t-t1)/(t2-t1);
%     xT = [polyval(s.y2,scaledt);polyval(s.z2,scaledt)];
%     dxT = 1/(t2-t1)*[polyval(s.dy2,scaledt);polyval(s.dz2,scaledt)];
%     d2xT = 1/(t2-t1)^2*[polyval(s.d2y2,scaledt);polyval(s.d2z2,scaledt)];
%     d3xT = 1/(t2-t1)^3*[polyval(s.d3y2,scaledt);polyval(s.d3z2,scaledt)];
%     d4xT = 1/(t2-t1)^4*[polyval(s.d4y2,scaledt);polyval(s.d4z2,scaledt)];
%     d5xT = 1/(t2-t1)^5*[polyval(s.d5y2,scaledt);polyval(s.d5z2,scaledt)];
%     d6xT = 1/(t2-t1)^6*[polyval(s.d6y2,scaledt);polyval(s.d6z2,scaledt)];    
else
    xT = [posY(numWaypoints, 1); posZ(numWaypoints, 1)];
    dxT = [ 0;0 ];
    d2xT = [ 0;0 ];
    d3xT = [ 0;0 ];
    d4xT = [ 0;0 ];
    d5xT = [ 0;0 ];
    d6xT = [ 0;0 ];

end


end
