% 7/16/13
% calculateInputs.m
% finds inputs to 2d quadrotor
% Dependancies: desiredTraj.m
%
% inputs: 
%   t: real value, current time
%   x: nx1 vector, current state
%   mQ, JQ: real values, constants
%   kp, kd, kp_phi, kd_phi: real values, gains
% outputs:
%   [f M]: 1x2 vector, control inputs for 2d quadrotor system at time t


function [f, M] = calculateInputs(t, x, g, mQ, JQ, kp, kd, kp_phi, kd_phi)
        % x = [xQ zQ vxQ vzQ phiQ phidotQ]'
        yQ = x(1, 1); zQ = x(2, 1); vyQ = x(3, 1); vzQ = x(4, 1); phiQ = x(5, 1); phidotQ = x(6, 1);
        
        % control laws
        [xT dxT d2xT d3xT d4xT] = desiredTraj(t, g, mQ, JQ);

        % find f input
        F1 =  mQ*g.*[0; 1] + mQ.*d2xT;
        F = -kp*([yQ; zQ] - xT) - kd*([vyQ; vzQ] - dxT) + F1;
        f = - F(1, 1)*sin(phiQ) + F(2, 1)*cos(phiQ);
        phiQ_des = atan2(-F(1, 1), F(2, 1));
        
        %calculate nominal f and phiQ_des values
        F_nom = mQ*d2xT + mQ.*g.*[0; 1];
        phiQ_nom = atan2(-F_nom(1, 1), F_nom(2, 1));
        f_nom = -F_nom(1, 1)*sin(phiQ_nom)+F_nom(2, 1)*cos(phiQ_nom);
        
        %use this to calculate phidotQ derivatives
        phidotQ_des = - mQ/f_nom * (d3xT(1, 1)*cos(phiQ_nom) + d3xT(2, 1)*sin(phiQ_nom));        
        phiddotQ_des = - mQ/f_nom * (d4xT(1, 1)*cos(phiQ_nom) + d4xT(2, 1)*sin(phiQ_nom)) ...
            - 2*mQ^2*phidotQ_des/f_nom^2 * (d2xT(1, 1)*d3xT(1, 1) + (d2xT(2, 1) + g) * d3xT(2, 1));

        M_des = JQ*phiddotQ_des;
        M = JQ* (-kp_phi*(phiQ-phiQ_des)-kd_phi*(phidotQ-phidotQ_des)) + M_des;



    % takes difference of two angles, keeping it between -pi and pi
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

