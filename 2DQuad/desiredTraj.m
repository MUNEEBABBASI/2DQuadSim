% 7/16/13
% desiredTraj.m
% desired quadrotor trajectory input
%
% inputs: 
%   t: real value, current time
%   g, mQ, JQ: real values, constants
% outputs:
%   xT, dxT, d2xT, d3xT, d4xT: each a 2x1 vector, position and higher derivatives of trajectory at current time

%%%%%
% Specify the position and derivatives of the desired trajectory
function [xT dxT d2xT d3xT d4xT] = desiredTraj(t, g, mQ, JQ)
% 
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

%     xT = [t; t];
%     vT = [1; 1];
%     aT = [0; 0];
%     jT = [0; 0];
%     sT = [0; 0];

xT = [t; t^2];
dxT = [1; 2*t];
d2xT = [0; 2];
d3xT = [0; 0];
d4xT = [0; 0];
d5xT = [0; 0];
d6xT = [0; 0];

end
