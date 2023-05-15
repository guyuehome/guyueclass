function U = skew(R)
%  Establish the skew symmetric matrix associated with the vector R  %
O       = zeros(3,3);
O(1,2)  = -R(3);
O(2,1)  = R(3);
O(1,3)  = R(2);
O(3,1)  = -R(2);
O(2,3)  = -R(1);
O(3,2)  = R(1);

U=O;
end