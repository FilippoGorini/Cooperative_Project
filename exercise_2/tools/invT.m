function T_inv = invT(T)
% INVT Computes the inverse of a homogeneous transformation matrix (4x4).
%   T_inv = invT(T) returns the inverse of T using the analytical formula:
%   
%   Given T = [ R  p ]
%             [ 0  1 ]
%
%   T_inv = [ R'  -R'*p ]
%           [ 0     1   ]
%
%   This is computationally faster and numerically more stable than 
%   using the generic inv(T) function.

    % Extract Rotation (R) and Position (p)
    R = T(1:3, 1:3);
    p = T(1:3, 4);

    % Compute the inverse Rotation (Transpose)
    R_inv = R';

    % Compute the inverse Position
    p_inv = -R_inv * p;

    % Construct the inverted matrix
    T_inv = [R_inv,    p_inv;
             0 0 0,    1    ];
end