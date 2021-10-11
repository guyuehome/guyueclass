
% Copyright (C) 1993-2015, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com
classdef VREP_mobile < handle
    
    properties
        vrep
        left
        right % VREP joint object handles
    end
    
    methods
        
        function mob = VREP_mobile(vrep, name)
            
            mob.vrep = vrep;
            
            
            % find all the _joint objects, we don't know how many joints so we
            % keep going till we get an error
            
            mob.left = vrep.gethandle('%s_leftJoint', name);
            mob.right = vrep.gethandle('%s_rightJoint', name);
            
            % set all joints to passive mode
            %             for j=1:arm.njoints
            %                 arm.vrep.simxSetJointMode(arm.client, arm.joint(j), arm.vrep.sim_jointmode_passive, arm.vrep.simx_opmode_oneshot_wait);
            %             end
        end
        
        function q = getq(mob)
            q = [
                mob.vrep.getjoint(mob.left)
                mob.vrep.getjoint(mob.right)
                ];
        end
        
        function setq(mob, q)
            mob.vrep.setjoint(mob.left, q(1));
            mob.vrep.setjoint(mob.right, q(2));
        end
        
        function setqd(mob, qd)
            mob.vrep.setjointvel(mob.left, qd(1));
            mob.vrep.setjointvel(mob.right, qd(2));
        end
    end
end
    
