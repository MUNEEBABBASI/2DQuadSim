% 7/26/13
% calculateInputs.m
% calculates inputs [f M] for quadrotor system with cable-suspended load
% dependancies: desiredTraj.m, calculateDerivatives.m
%
% inputs: 
%   t: real value, current time
%   x1: nx1 vector, current state 
%   g, mL, mQ, jQ, l: real values, constants
%   kpx, kdx, kpL, kdL, kpQ, kdQ: real values, control gains
% outputs: 
%   [f M]: 1x2 vector, control inputs for 2d quadrotor system at time t


function [f, M, phiL_des, phiQ_des, d2phiQ_nom, p_des, dp_nom] = calculateInputs1(t, x1, g, mL, mQ, JQ, l, kpx, kdx, kpL, kdL, kpQ, kdQ)

% x1 = [xL vL phiL phidotL phiQ phidotQ]', note xL and vL are vectors in R^2
yL = x1(1, 1); zL = x1(2, 1); vyL = x1(3, 1); vzL = x1(4, 1); phiL = x1(5, 1); phidotL = x1(6, 1); ...
    phiQ = x1(7, 1); phidotQ = x1(8, 1);


%find desired trajectory and its higher derivatives
[xT, dxT, d2xT, d3xT, d4xT, d5xT, d6xT] = desiredTraj(t, g, mQ, JQ, 1);

% find nominal p, phiL, phiQ, f, higher derivatives
[p_nom, dp_nom, d2p_nom, d3p_nom, d4p_nom, ...
    phiL_nom, dphiL_nom, d2phiL_nom, d3phiL_nom, d4phiL_nom, ...
    f_nom, phiQ_nom, dphiQ_nom, d2phiQ_nom] = calculateDerivatives(t, g, mL, mQ, JQ, l);


%%%
%load position control
%FA = -kpx.*([yL;zL]-xT) - kdx.*([vyL; vzL]-dxT) + mL.*d2xT + mL.*g.*[0; 1];
FA = -kpx.*([yL;zL]-xT) - kdx.*([vyL; vzL]-dxT) + d2xT + g.*[0; 1]
p_des = - FA./norm(FA)
phiL_des = atan2(-FA(1, 1), FA(2, 1))

FB = mQ.*d2xT-mQ.*l.*d2p_nom+mQ.*g.*[0;1]
F = (mL*FA+FB)
phiQ
f = -F(1, 1)*sin(phiQ) + F(2, 1)*cos(phiQ)

%%%
%load attitude control
%asinterm = -kpL*(angleDiff(phiL, phiL_des))-kdL*(phidotL-dphiL_nom)+(d2phiL_nom*mQ*l/f);
%phiQ_des = phiL_nom + asin(asinterm);
asinterm = d2phiL_nom*mQ*l/f;
phiQ_des = phiL_des -(kpL-1)*(angleDiff(phiL, phiL_des))-kdL*(phidotL-dphiL_nom)+ asin(asinterm);

% check if asin term is greater than 1 (causes imaginary values in states
if (abs(asinterm) > 1)
    error('greater than 1')
end
 
 
%%%
%quadrotor attitude control
M = JQ*(-kpQ*(phiQ-phiQ_des) - kdQ*(phidotQ-dphiQ_nom)) + JQ*d2phiQ_nom;




    % takes difference of two angles, bounding output between -pi and pi
    function diff = angleDiff(x1, x2)
        diff = x1-x2;
        
        while diff > pi
            diff = diff - 2*pi;
        end
        
        while diff <= -pi
            diff = diff + 2*pi;
        end
    end


end

