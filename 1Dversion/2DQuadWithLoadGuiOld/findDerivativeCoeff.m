% 7/30/13
% findDerivativeCoeff.m
% finds coefficients for up to the kth derivative of polynominals of order n
% inputs:
%   n: integer, order of desired polynominal trajectory
%   r: integer, derivative to find up to (and including)
% outputs:
%   derCoeff: (r+1) x (n+1) matrix, row i contains coefficients for the
%       (i-1)th derivative of a polynominal of order n, assuming all
%       position coefficients are 1
function derCoeff = findDerivativeCoeff(n, r)

derCoeff = zeros(r+1, n+1);
tempC = ones(1, n+1); % constant
derCoeff(1, :) = tempC;
for i = 1:r, %for derivatives 1 to k
    tempC = polyder(tempC);
    derCoeff(i+1, :) = [tempC zeros(1, n+1-length(tempC))];
end

end