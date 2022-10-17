function [ arm, elbow, wrist ] = Configuration( rconf )
%RCONF Summary of this function goes here
%   Detailed explanation goes here

arm = 1;
elbow = 1;
wrist = 1;

if(bitand(rconf,1))
   arm = -1; 
end

if(bitand(rconf,2))
   elbow = -1; 
end

if(bitand(rconf,4))
   wrist = -1; 
end


end

