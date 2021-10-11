%ICP Point cloud alignment
%
% T = ICP(P1, P2, OPTIONS) is the homogeneous transformation that best
% transforms the set of points P1 to P2 using the iterative closest point
% algorithm.
%
% [T,D] = ICP(P1, P2, OPTIONS) as above but also returns the norm of the
% error between the transformed point set P2 and P1.
%
% Options::
% 'dplot',D        show the points P1 and P2 at each iteration, with a
%                  delay of D [sec].
% 'plot'           show the points P1 and P2 at each iteration, with a
%                  delay of 0.5 [sec].
% 'maxtheta',T     limit the change in rotation at each step 
%                  to T (default 0.05 rad)
% 'maxiter',N      stop after N iterations (default 100)
% 'mindelta',T     stop when the relative change in error norm is less 
%                  than T (default 0.001)
% 'distthresh',T   eliminate correspondences more than T x the median distance
%                  at each iteration.
%
% Example::
% Create a 3D point cloud
%         p1 = randn(3,20);                                       
% Transform it by an arbitrary amount
%         T = transl(1,2,3)*eul2tr(0.1, 0.2, 0.3)
%         p2 = homtrans( T, p1);
% Perform ICP to determine the transformation that maps p1 to p2
%         icp(p1, p2)
%
% Notes::
% - Does not require knowledge of correspondence between the points.
%   - The point sets may have different numbers of points.
%   - Points in either set may have no corresponding point.
% - Points can be 2- or 3-dimensional.
% - For noisy data setting distthresh and maxtheta can help to prevent the
%   solution from diverging.
%
% Reference::
% - "A method for registration of 3D shapes",
%   P.Besl and H.McKay,
%   IEEETrans. Pattern Anal. Mach. Intell., vol. 14, no. 2,
%   pp. 239-256, Feb. 1992.



% Copyright (C) 1993-2011, by Peter I. Corke
%
% This file is part of The Machine Vision Toolbox for Matlab (MVTB).
% 
% MVTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% MVTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with MVTB.  If not, see <http://www.gnu.org/licenses/>.
function [T,d] = icp(set1, set2, varargin)

    if numrows(set1) ~= numrows(set2)
        error('point clouds must have matching point dimensions');
    end
    ndims = numrows(set1);
    if ndims < 2 || ndims > 3
        error('ICP only for 2 or 3 dimensional points');
    end

    % T is the transform from set1 to set2    
    opt.maxtheta = []; %0.05;
    opt.maxiter = 100;
    opt.mindelta = 0.001;
    opt.plot = false;
    opt.dplot = 0;
    opt.distthresh = [];
    opt = tb_optparse(opt, varargin);
    
    if opt.plot
        opt.dplot = 0.5;
    end
    
    
	N1 = numcols(set1);	% number of model points
	N2 = numcols(set2);

	p1 = mean(set1');
	p2 = mean(set2');
	t = p2 - p1;
	T = transl(t);
    
	dnorm = 0;
    dnorm_p = NaN;
    
	for count=1:opt.maxiter
        
		% transform the model
        set1_tr = homtrans(T, set1);

		% for each point in set 2 find the nearest point in set 1       
        [corresp,distance] = closest(set2, set1_tr);
        
        % optionally break correspondences if their distance is above some
        % multiple of the median distance
        if isempty(opt.distthresh)
            set2_tr = set2;
        else
            k = find(distance > opt.distthresh*median(distance));

            % now remove them
            if ~isempty(k),
                %fprintf('breaking %d corespondences ', length(k));
                distance(k) = [];
                corresp(k) = [];
            end
            set2_tr = set2;
            set2_tr(:,k) = [];
        end
        
		% display the model points and correspondences
        if opt.dplot > 0
            usefig('ICP');
            clf
            if ndims == 2
                plot(set1_tr(1,:),set1_tr(2,:),'bx');
                grid
                hold on
                plot(set2_tr(1,:),set2_tr(2,:),'ro');
                
                for i=1:numcols(set2_tr),
                    ic = corresp(i);
                    plot( [set1_tr(1,ic) set2_tr(1,i)], [set1_tr(2,ic) set2_tr(2,i)], 'g');
                end
            else
                plot3(set1_tr(1,:),set1_tr(2,:),set1_tr(3,:),'bx');
                grid
                hold on
                plot3(set2_tr(1,:),set2_tr(2,:),set2_tr(3,:),'ro');
                
                for i=1:numcols(set2_tr),
                    ic = corresp(i);
                    plot3( [set1_tr(1,ic) set2_tr(1,i)], [set1_tr(2,ic) set2_tr(2,i)], [set1_tr(3,ic) set2_tr(3,i)], 'g');
                end
            end
            pause(opt.dplot)
        end

		% find the centroids of the two point sets
		% for the observations include only those points which have a
		% correspondance.
		p1 = mean(set1_tr(:,corresp)');
        %[length(corresp)         length(unique(corresp))]
        %p1 = mean(set1_tr');
		p2 = mean(set2_tr');

        % compute the moments
		M = zeros(ndims, ndims);
		for i=1:numcols(set2_tr)
            ic = corresp(i);
			M = M + (set1_tr(:,ic) - p1') * (set2_tr(:,i) - p2')';
        end
        
		[U,S,V] = svd(M);
        
        % compute the rotation of p1 to p2
        % p2 = R p1 + t
		R = V*U';

        if det(R) < 0
            warning('rotation is not in SO(3)');
            R = -R;
        end
        
        if opt.debug
            p1
            p2
            M
            R
        end
        
        % optionally clip the rotation, helps converence
		if ~isempty(opt.maxtheta)
			[theta,v] = tr2angvec(R);
            if theta > opt.maxtheta;
                theta = opt.maxtheta;
            elseif theta < -opt.maxtheta
                theta = -opt.maxtheta;
            end
            R = angvec2r(theta, v);
        end
        if opt.debug
            theta
            v
            R
        end

			
		% determine the incremental translation
        t = p2' - p1';
		
		%disp([t' tr2rpy(R)])

		% update the transform from data (observation) to model
		T = trnorm( T * rt2tr(R, t) );
		%count = count + 1;
		rpy = tr2rpy(T);
		
        dnorm = norm(distance);
        
        if opt.verbose
            if ndims == 2
                fprintf('[%d]: n=%d/%d, d=%8.3f, t = (%8.3f %8.3f), th = (%6.1f) deg\n', ...
                    count, length(distance), numcols(set2), dnorm, transl(T), atan2(T(2,1), T(2,2))*180/pi);
            elseif ndims == 3
                fprintf('[%d] n=%d/%d, d=%8.3f, t = (%8.3f %8.3f %8.3f), rpy = (%6.1f %6.1f %6.1f) deg\n', ...
                    count, length(distance), numcols(set2), dnorm, transl(T), rpy*180/pi);
            end
            %std(distance)
        end

        % check termination condition
        if abs(dnorm - dnorm_p)/dnorm_p < opt.mindelta
            count = NaN;    % flag that we exited on mindelta
            break
        end
		dnorm_p = dnorm;
    end
    
    if opt.verbose
        if isnan(count)
                fprintf('terminate on minimal change of error norm');
        else
            fprintf('terminate on iteration limit (%d iterations)\n', opt.maxiter);
        end
    end
    
    if nargout > 1
        d = dnorm;
    end
