% 7/31/13
% desiredTraj.m
% desired quadrotor trajectory input
% Dependencies: optTraj.m
%
% inputs: 
%   t: real value, current time
% outputs:
%   xT: k x d x m matrix, where the d columns each contain value of a
%   dimension (for example, x, y, z position) at time t, the k rows contain
%   values of the (i-1)th derivative of dimension j at time t, and the m of
%   the kxd matrices each represent a piece of the piecewise trajectory


%%%%%
% Specify the position and derivatives of the desired trajectory
function [xT] = desiredTraj(t)


posT = [0; 2; 4];
posY = [0 1 1 ; 0 -1 0; 0 -1 0; 0 -1 0; 0 -1 0; 0 -1 0];
posZ = [0 0 2 ; 0 -1 0; 0 -1 0; 0 -1 0; 0 -1 0; 0 -1 0];

% posT = [0; 3];
% posY = [0 4; 0 0; 0 0; 0 0; 0 0; 0 0];
% posZ = [0 0; 0 0; 0 0; 0 0; 0 0; 0 0];

% posT = [0; 4;8;12];
% posY = [0 2 4 6; 0 -1 -1 0; 0 -1 -1 0; 0 -1 -1 0; 0 -1 -1 0; 0 -1 -1 0];
% posZ = [0 3 -2 1; 0 -1 -1 0; 0 -1 -1 0; 0 -1 -1 0; 0 -1 -1 0; 0 -1 -1 0];


numWaypoints = length(posT);

smooth = 1;

ineqConst.start = 2;
ineqConst.nc = 1;
ineqConst.delta = 0.05;

% optimize a trajectory between these waypoints

if(isempty(traj))

    % joint optimization if more than one waypoint
    if (numWaypoints > 2 && smooth == 1),
        [yCoeff, zCoeff] = optTrajCorr(posT, posY, posZ, ineqConst);
        %yCoeff = optTrajJoint(posT, posY);
        %zCoeff = optTrajJoint(posT, posZ);
    
        for i = 1:numWaypoints -1,
        traj.y{i} = yCoeff(:, i);
        traj.z{i} = zCoeff(:, i);
        end
    
    % otherwise, just fix the ends
    else
        for i = 1:numWaypoints-1,
        traj.y{i} = optTraj([posY(1 ,i),0,0,0,0,0,0,0]',[posY(1, i+1),0,0,0,0,0,0,0]');
        traj.z{i} = optTraj([posZ(1, i),0,0,0,0,0,0,0]',[posZ(1, i+1),0,0,0,0,0,0,0]');
        end
    end
    
    for i = 1:numWaypoints-1,
       % traj.y{i} = optTraj([posY(i, 1),0,0,0,0,0,0,0]',[posY(i+1, 1),0,0,0,0,0,0,0]');
      %  traj.z{i} = optTraj([posZ(i, 1),0,0,0,0,0,0,0]',[posZ(i+1, 1),0,0,0,0,0,0,0]');
        
        traj.dy{i} = polyder(traj.y{i});
        traj.d2y{i} = polyder(traj.dy{i});
        traj.d3y{i} = polyder(traj.d2y{i});
        traj.d4y{i} = polyder(traj.d3y{i});
        traj.d5y{i} = polyder(traj.d4y{i});
        traj.d6y{i} = polyder(traj.d5y{i});
        
        traj.dz{i} = polyder(traj.z{i});
        traj.d2z{i} = polyder(traj.dz{i});
        traj.d3z{i} = polyder(traj.d2z{i});
        traj.d4z{i} = polyder(traj.d3z{i});
        traj.d5z{i} = polyder(traj.d4z{i});
        traj.d6z{i} = polyder(traj.d5z{i});
        
        traj.posT = posT;
        traj.posY = posY;
        traj.posZ = posZ;

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
            xT = [polyval(traj.y{i},scaledt);polyval(traj.z{i},scaledt)];
            dxT = 1/(posT(i+1, 1)-posT(i, 1))*[polyval(traj.dy{i},scaledt);polyval(traj.dz{i},scaledt)];
            d2xT = 1/(posT(i+1, 1)-posT(i, 1))^2*[polyval(traj.d2y{i},scaledt);polyval(traj.d2z{i},scaledt)];
            d3xT = 1/(posT(i+1, 1)-posT(i, 1))^3*[polyval(traj.d3y{i},scaledt);polyval(traj.d3z{i},scaledt)];
            d4xT = 1/(posT(i+1, 1)-posT(i, 1))^4*[polyval(traj.d4y{i},scaledt);polyval(traj.d4z{i},scaledt)];
            d5xT = 1/(posT(i+1, 1)-posT(i, 1))^5*[polyval(traj.d5y{i},scaledt);polyval(traj.d5z{i},scaledt)];
            d6xT = 1/(posT(i+1, 1)-posT(i, 1))^6*[polyval(traj.d6y{i},scaledt);polyval(traj.d6z{i},scaledt)];

%             scaledt = t;
%             xT = [polyval(traj.y{i},scaledt);polyval(traj.z{i},scaledt)];
%             dxT = [polyval(traj.dy{i},scaledt);polyval(traj.dz{i},scaledt)];
%             d2xT = [polyval(traj.d2y{i},scaledt);polyval(traj.d2z{i},scaledt)];
%             d3xT = [polyval(traj.d3y{i},scaledt);polyval(traj.d3z{i},scaledt)];
%             d4xT = [polyval(traj.d4y{i},scaledt);polyval(traj.d4z{i},scaledt)];
%             d5xT = [polyval(traj.d5y{i},scaledt);polyval(traj.d5z{i},scaledt)];
%             d6xT = [polyval(traj.d6y{i},scaledt);polyval(traj.d6z{i},scaledt)];
% 
%             
            return
        end
    end  
else
    xT = [posY(1, numWaypoints); posZ(1, numWaypoints)];
    dxT = [ 0;0 ];
    d2xT = [ 0;0 ];
    d3xT = [ 0;0 ];
    d4xT = [ 0;0 ];
    d5xT = [ 0;0 ];
    d6xT = [ 0;0 ];

end




end


