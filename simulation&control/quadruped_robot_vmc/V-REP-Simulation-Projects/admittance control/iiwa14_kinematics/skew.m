function skew_matrix = skew(v)
% SKEW
%
% Converts a 3-D vector to a skew matrix
% Input:  v - 3-D vector
% Output: skew_matrix - skew matrix

skew_matrix = [0 -v(3) v(2);
               v(3) 0 -v(1);
               -v(2) v(1) 0];
end