% 7/22/13
% calculateDerivatives.m
% calculates derivatives of states of quadrotor with (taut) cable-suspended load system
% dependencies: desiredTraj.m
%
% inputs:
%   t: real value, current time
%   g, mL, mQ, JQ, l: real values, constants
% outputs:
%   p, dp, d2p, d3p, d4p: 2x1 vector, p and its higher derivatives
%   phiL, dphiL, d2phiL, d3phiL, d4phiL: real values, phiL and its higher derivatives 
%   f: real value, desired thrust force without feedback control
%   phiQ, dphiQ, d2phiQ: real values, phiQ and its higher derivatives

%%%%%
% Specify the position and derivatives of the desired trajectory
function [p, dp, d2p, d3p, d4p, phiL, dphiL, d2phiL, d3phiL, d4phiL, f, phiQ, dphiQ, d2phiQ] = calculateDerivatives(t, g, mL, mQ, JQ, l)

    % find desired trajectory and its derivatives
    [xT, dxT, d2xT, d3xT, d4xT, d5xT, d6xT] = desiredTraj(t, g, mQ, JQ);

    %calculate derivatives of T, p, phiL
    Tp = -(mL.*d2xT + mL.*g.*[0;1]);
    p = Tp./ norm(Tp);
    phiL = atan2(Tp(1, 1), -Tp(2, 1));
    T = Tp(1, 1)*sin(phiL) - Tp(2, 1)*cos(phiL);
    
    dT = mL*(d2xT(1, 1)*d3xT(1, 1) + (d2xT(2, 1)+g)*d3xT(2, 1))*(d2xT(1, 1)^2+(d2xT(2, 1)+g)^2)^(-1/2);
    dp = -1./T.*(mL.*d3xT + dT.*p);
    dphiL = dp(1, 1)*cos(phiL) + dp(2, 1)*sin(phiL);
    
    
    % calculate higher derivatives of T, p, phiL
    d2T = mL*( ...
        ( (d3xT(1, 1)^2+d2xT(1, 1)*d4xT(1, 1)+d4xT(2, 1)*(d2xT(2, 1)+g)+d3xT(2, 1)^2) * (d2xT(1, 1)^2+(d2xT(2, 1)+g)^2)^(-1/2) )...
        - ( (d2xT(1, 1)*d3xT(1, 1)+(d2xT(2, 1)+g)*d3xT(2, 1))^2 * (d2xT(1, 1)^2+(d2xT(2, 1)+g)^2)^(-3/2) ) );
    d2p = -1./T.*(mL.*d4xT + d2T.*p + 2.*dT.*dp);
    d2phiL = d2p(1, 1)*cos(phiL) + d2p(2, 1)*sin(phiL);
    
    d3T = ( mL* ( ...
        ( 3*(2*d3xT(2, 1)*(d2xT(2, 1)+g)+2*d3xT(1, 1)*d2xT(1, 1))^3 ) ...
        - (12*((d2xT(2, 1)+g)^2+d2xT(1, 1)^2)*(d4xT(2, 1)*(d2xT(2, 1)+g)+d3xT(1, 1)^2+d4xT(1, 1)*d2xT(1, 1)+d3xT(2, 1)^2)*(2*d3xT(2, 1)*(d2xT(2, 1)+g)+2*d3xT(1, 1)*d2xT(1, 1))) ...
        + (4*((d2xT(2, 1)+g)^2+d2xT(1, 1)^2)^2*(2*d5xT(2, 1)*(d2xT(2, 1)+g)+2*d5xT(1, 1)*d2xT(1, 1)+6*d3xT(1, 1)*d4xT(1, 1)+6*d3xT(2, 1)*d4xT(2, 1))) ) ...
        ) ...
        * ( ...
            8*((d2xT(2, 1)+g)^2+d2xT(1, 1)^2)^(5/2) )^(-1);
    d3p = -1./T.*(mL.*d5xT + 3*d2T*dp + 3*dT*d2p + d3T*p);
    d3phiL = d3p(1, 1)*cos(phiL) + d3p(2, 1)*sin(phiL) + dphiL^3;
    
    d4T = ( mL* ( ...
        - (15*(2*d3xT(2, 1)*(d2xT(2, 1)+g)+2*d3xT(1, 1)*d2xT(1, 1))^4) ...
        + (72*((d2xT(2, 1)+g)^2+d2xT(1, 1)^2)*(d4xT(2, 1)*(d2xT(2, 1)+g)+d3xT(1, 1)^2+d4xT(1, 1)*d2xT(1, 1)+d3xT(2, 1)^2)*(2*d3xT(2, 1)*(d2xT(2, 1)+g)+2*d3xT(1, 1)*d2xT(1, 1))^2) ...
        - (48*((d2xT(2, 1)+g)^2+d2xT(1, 1)^2)^2*(d4xT(2, 1)*(d2xT(2, 1)+g)+d3xT(1, 1)^2+d4xT(1, 1)*d2xT(1, 1)+d3xT(2, 1)^2)^2) ...
        - (16*((d2xT(2, 1)+g)^2+d2xT(1, 1)^2)^2*(2*d5xT(2, 1)*(d2xT(2, 1)+g)+2*d5xT(1, 1)*d2xT(1, 1)+6*d3xT(1, 1)*d4xT(1, 1)+6*d3xT(2, 1)*d4xT(2, 1))*(2*d3xT(2, 1)*(d2xT(2, 1)+g)+2*d2xT(1, 1)*d3xT(1, 1))) ...
        + (8*((d2xT(2, 1)+g)^2+d2xT(1, 1)^2)^3*(2*d6xT(2, 1)*(d2xT(2, 1)+g)+6*d4xT(1, 1)^2+2*d6xT(1, 1)*d2xT(1, 1)+8*d3xT(1, 1)*d5xT(1, 1)+6*d4xT(2, 1)^2+8*d3xT(2, 1)*d5xT(2, 1))) ) ...
        ) ...
        * ( ...
            16*((d2xT(2, 1)+g)^2+d2xT(1, 1)^2)^(7/2) )^(-1);
    d4p = -1./T.*(mL.*d6xT + 4.*d3T.*dp + 6.*d2T.*d2p + 4.*dT.*d3p + d4T*p);
    d4phiL = d4p(1, 1)*cos(phiL) + d4p(2, 1)*sin(phiL) + 6*dphiL^2*d2phiL;

    
    
    % calculate f, phiQ and their higher derivatives   
    F = (mQ+mL)*(d2xT + g.*[0; 1]) - mQ*l*d2p;
    phiQ = atan2(-F(1, 1), F(2, 1));
    f = - F(1, 1)*sin(phiQ) + F(2, 1)*cos(phiQ);
    
    dphiQ = ( ((l*mQ*d3p(1, 1)-(mQ+mL)*d3xT(1, 1)) * ((mQ+mL)*(d2xT(2, 1)+g)-l*mQ*d2p(2, 1))) ...
        + ((l*mQ*d3p(2, 1)-(mQ+mL)*d3xT(2, 1)) * (l*mQ*d2p(1, 1)-(mQ+mL)*d2xT(1, 1))) ) ...
        * ( (l*mQ*d2p(2, 1)-(mQ+mL)*(d2xT(2, 1)+g))^2 * ((l*mQ*d2p(1, 1)-(mQ+mL)*d2xT(1, 1))^2/((l*mQ*d2p(2, 1)-(mQ+mL)*(d2xT(2, 1)+g))^2) + 1) )^-1; 
    
    
    d2phiQ = ( ...
            ( ((l*mQ*d4p(1, 1)-(mQ+mL)*d4xT(1, 1))*((mQ+mL)*(d2xT(2, 1)+g)-l*mQ*d2p(2, 1))^(-1)) ...
             -(2*(l*mQ*d3p(1, 1)-(mQ+mL)*d3xT(1, 1))*((mQ+mL)*d3xT(2, 1)-l*mQ*d3p(2, 1))*((mQ+mL)*(d2xT(2, 1)+g)-l*mQ*d2p(2, 1))^(-2)) ...
             -(((mQ+mL)*d4xT(2, 1)-l*mQ*d4p(2, 1))*(l*mQ*d2p(1, 1)-(mQ+mL)*d2xT(1, 1))*((mQ+mL)*(d2xT(2, 1)+g)-l*mQ*d2p(2, 1))^(-2)) ...
             +(2*((mQ+mL)*d3xT(2, 1)-l*mQ*d3p(2, 1))^2*(l*mQ*d2p(1, 1)-(mQ+mL)*d2xT(1, 1))*((mQ+mL)*(d2xT(2, 1)+g)-l*mQ*d2p(2, 1))^(-3)) ) ...
             * ( (((l*mQ*d2p(1, 1)-(mQ+mL)*d2xT(1, 1))^2*((mQ+mL)*(d2xT(2, 1)+g)-l*mQ*d2p(2, 1))^(-2))+1)^(-1) ) ...
             ) ...
             - ( ...
            ( (2*(l*mQ*d3p(1, 1)-(mQ+mL)*d3xT(1, 1))*(l*mQ*d2p(1, 1)-(mQ+mL)*d2xT(1, 1))*((mQ+mL)*(d2xT(2, 1)+g)-l*mQ*d2p(2, 1))^(-2)) ...
             -(2*((mQ+mL)*d3xT(2, 1)-l*mQ*d3p(2, 1))*(l*mQ*d2p(1, 1)-(mQ+mL)*d2xT(1, 1))^2*((mQ+mL)*(d2xT(2, 1)+g)-l*mQ*d2p(2, 1))^(-3)) ) ...
             * ( ((l*mQ*d3p(1, 1)-(mQ+mL)*d3xT(1, 1))*((mQ+mL)*(d2xT(2, 1)+g)-l*mQ*d2p(2, 1))^(-1)) ...
             -(((mQ+mL)*d3xT(2, 1)-l*mQ*d3p(2, 1))*(l*mQ*d2p(1, 1)-(mQ+mL)*d2xT(1, 1))*((mQ+mL)*(d2xT(2, 1)+g)-l*mQ*d2p(2, 1))^(-2)) ) ...
             * ( ((l*mQ*d2p(1, 1)-(mQ+mL)*d2xT(1, 1))^2*((mQ+mL)*(d2xT(2, 1)+g)-l*mQ*d2p(2, 1))^(-2))+1)^(-2) );

         
%          
%          % Correction for potential 0/0
%          epsilon = 5e-2 ;
%          ind = find(abs(d2xT(1, 1))<epsilon & abs(d2xT(2, 1) + g)<epsilon) ;
%          if(length(ind))
%              x = 1:length(d2xT) ;
%              x = setdiff(x, ind) ;
%              p(ind) = interp1(x,p(x),ind,'linear','extrap') ;
%              dp(ind) = interp1(x,dp(x),ind,'linear','extrap') ;
%              d2p(ind) = interp1(x,d2p(x),ind,'linear','extrap') ;
%              d3p(ind) = interp1(x,d3p(x),ind,'linear','extrap') ;
%              d4p(ind) = interp1(x,d4p(x),ind,'linear','extrap') ;
%              
%              phiL(ind) = interp1(x,phiL(x),ind,'linear','extrap') ;
%              dphiL(ind) = interp1(x,dphiL(x),ind,'linear','extrap') ;
%              d2phiL(ind) = interp1(x,d2phiL(x),ind,'linear','extrap') ;
%              d3phiL(ind) = interp1(x,d3phiL(x),ind,'linear','extrap') ;
%              d4phiL(ind) = interp1(x,d4phiL(x),ind,'linear','extrap') ;
%              
%              f(ind) = interp1(x,f(x),ind,'linear','extrap') ;
%              phiQ(ind) = interp1(x,phiQ(x),ind,'linear','extrap') ;            
%              dphiQ(ind) = interp1(x,dphiQ(x),ind,'linear','extrap') ;              
%              d2phiQ(ind) = interp1(x,d2phiQ(x),ind,'linear','extrap') ;              
%              
%          end

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
