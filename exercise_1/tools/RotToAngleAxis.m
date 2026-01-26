function [h,theta] = RotToAngleAxis(R)
%EULER REPRESENTATION: Given a tensor rotation matrices this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'h' (axis) 
% SUGGESTED FUNCTIONS
    % size()
    % eye()
    % abs()
    % det()
    % NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
    % and axis of rotation. Error messages must be displayed if the matrix
    % does not satisfy the rotation matrix criteria.

    % Check if R is a rotational matrix
    tolerance = 1e-6;
    if (~isequal(size(R), [3, 3]) || abs(det(R) - 1) > tolerance || norm(inv(R) - R', 'fro') > tolerance)
        error('Input matrix is not a rotational matrix');
    end

    % Computes the angle of rotation
    theta = acos((trace(R)-1)/2);

    % Computes the axis vector
    if (theta == 0)
        warning('h is arbitrary');
        h = [0 0 1]';

    elseif (theta == pi)
        h = zeros(3,1);
        h(1) = sqrt((R(1,1) + 1)/2);
        if (h(1) == 0)
            h(2) = sqrt((R(2,2) + 1)/2);
            if (h(2) == 0)
                h(3) = sqrt((R(3,3) + 1)/2);
                if (h(3) == 0)
                    warning('There is no rotation.')
                end
            else
                h(3) = sign(R(3,3)) * sqrt((R(3,3) + 1)/2);
            end
        else
            h(2) = sign(R(1,2)) * sqrt((R(2,2) + 1)/2);
            h(3) = sign(R(1,3)) * sqrt((R(3,3) + 1)/2);
        end

    elseif (0 < theta) && (theta < pi)
        h = (vex((R-R')/2))./sin(theta);
    end

end


function a = vex(S_a)
    a = [S_a(3,2), S_a(1,3), S_a(2,1)]';  
end