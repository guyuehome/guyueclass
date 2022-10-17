function unit_vector = unit(v)
% UNIT
% 
% Normalizes a vector
% Input:  v - a vector
% Output: unit_vector - normalized vector

 unit_vector = v/norm(v);
end