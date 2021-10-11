%KMEANS K-means clustering
%
% [L,C] = KMEANS(X, K, OPTIONS) is a K-means clustering of multi-dimensional 
% data points X (DxN) where N is the number of points, and D is the dimension.
% The data is organized into K clusters based on Euclidean distance from cluster
% centres C (DxK). L is a vector (Nx1) whose elements indicates which 
% cluster the corresponding element of X belongs to.  
%
% [L,C] = KMEANS(X, K, C0) as above but the initial clusters C0 (DxK) is given
% and column I is the initial estimate of the centre of cluster I.
%
% L = KMEANS(X, C) is similar to above but the clustering step is not performed,
% it is assumed to have been completed previously.  C (DxK) contains the cluster
% centroids and L (Nx1) indicates which cluster the corresponding element of X
% is closest to.
%
% Options::
% 'random'   initial cluster centres are chosen randomly from the set of
%            data points X (default)
% 'spread'   initial cluster centres are chosen randomly from within the 
%            hypercube spanned by X.
%
% Reference::
% "Pattern Recognition Principles",
% Tou and Gonzalez,
% Addison-Wesley 1977, pp 94


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

% add Kaufman initialization
%
% 1) Initialize c1 with the most centrally located input sample. 
% 2) For i = 2, . . . , K, each ci is initialized in the following 
% way: for each non-selected input sample xj , calculate its 
% summed distance to the other non-selected input samples xl , 
% who are closer to xj than to their respective nearest seed 
% clusters, 
% http://www.comp.nus.edu.sg/~tancl/Papers/IJCNN04/he04ijcnn.pdf
% http://citeseer.ist.psu.edu/398385.html

function [label,centroid,resid] = kmeans(x, K, varargin)
% TODO update doco for assignment mode
%      option to loop N times and take lowest residual
%      return residual from assignment mode

    n = numcols(x);
    
    if numcols(K) > 1 && numrows(x) == numrows(K)
        % kmeans(x, centres)
        % then return closest clusters
        label = closest(x, K);
        if nargout > 1
            centroid = K;
        end
        if nargout > 2
            resid = norm( x - K(:,label) );
        end
        return
    end

    opt.plot = false;
    opt.init = {'random', 'spread'};
    [opt,z0] = tb_optparse(opt, varargin);

    if opt.plot && numrows(x) > 3
        warning('cant plot for more than 3D data');
        opt.plot = false;
    end
    if ~isempty(z0)
        z0 = z0{1};
        % an initial condition was supplied
        if numcols(z0) ~= K
            error('initial cluster length should be k');
        end
        if numrows(z0) ~= numrows(x)
            error('number of dimensions of z0 must match dimensions of x');
        end
    else
        % determine initial condition
        if strcmp(opt.init, 'random')
            % select K points from the set given as initial cluster centres
            k = randi(n, K, 1);
            z0 = x(:,k);
        elseif strcmp(opt.init, 'spread')
            % select K points from within the hypercube defined by the points
            mx = max(x')';
            mn = min(x')';
            z0 = (mx-mn) * rand(1,K) + mn * ones(1,K);
        end
    end

    % z is the centroid
    % zp is the previous centroid
    % s is the vector of cluster labels corresponding to rows in x
    
    z = z0;
    z

    %
    % step 1
    %
    zp = z;             % previous centroids
    s = zeros(1,n);
    sp = s;
    
    iterating = true;
    k = 1;
    iter = 0;
    
    while iterating
        iter = iter + 1;
        
        tic
        t0 = toc;

        %
        % step 2
        %

        s = closest(x, z);
                    
        %
        % step 3
        %
        for j=1:K
            k = find(s==j);
            if isempty(k)
                if strcmp(opt.init, 'random')
                    k = randi(n, 1, 1);
                    z0 = x(:,k);
                elseif strcmp(opt.init, 'spread')
                    % this cluster has no elements, randomly assign it
                    zp(:,j) = (mx-mn) * rand(1,1) + mn;
                end
            else
                % else, assign it to the mean of its elements
                zp(:,j) = mean( x(:,k)' );
                dd = sum( (repmat(zp(:,j),1,length(k)) - x(:,k)).^2 );
                dd = dd / numrows(x);
                maxerr(j) = max( sqrt(dd) );
            end
        end

        %
        % step 4
        %
        %  determine the change in cluster centres over the last step
        delta = sum( sum( (z - zp).^2 ) );

        if opt.verbose
            t = toc;
            fprintf('%d: norm=%g, delta=%g, took %.1f seconds\n', iter, mean(maxerr), delta, t);
        end

        if delta == 0
            %fprintf('delta to zero\n');
            iterating = false;
        end
        z = zp;
        if opt.plot
            if numrows(z) == 2
                plot(z(1,:), z(2,:));
            else
                plot3(z(1,:), z(2,:), z(3,:));
            end
            pause(.1);
        end
        
        % if no point assignments changed then we are done
        if all(sp == s)
            %fprintf('no point assignments changed\n');
            iterating = 0;
        end
        sp = s;
    end
    
    if nargout == 0
        % if no output arguments display results        
        if numrows(z) < 5
            for i=1:K,
                fprintf('cluster %d: %s (%d elements)\n', i, ...
                    sprintf('%11.4g ', z(i,:)), length(find(s==i)));
            end
        end
        
    end
    if nargout > 0
        centroid = z;
    end
    if nargout > 1
        label = s;
    end
    if nargout > 2
        % compute the residual
        resid = norm( x - z(:,s) );
    end

    dt = toc - t0;
    if dt > 10
        fprintf('that took %.1f seconds\n', dt);
    elseif dt > 60
        fprintf('that took %.1f minutes\n', dt/60);
    end
