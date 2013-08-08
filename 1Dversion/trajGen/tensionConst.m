
function [c,ceq]=tensionConst(x)

global options
d = options.d;
pos = zeros(d, 1);

for i = 1:d,
    pos(i, 1) = x(i, 1);
end

T = norm( pos + [0; 9.81] );

%c = -T-options.gamma;
%ceq = [];

c = [];
ceq = T;

end