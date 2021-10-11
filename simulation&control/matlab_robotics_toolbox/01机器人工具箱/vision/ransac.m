%RANSAC Random sample and consensus
%
% M = RANSAC(FUNC, X, T, OPTIONS) is the RANSAC algorithm that robustly fits
% data X to the model represented by the function FUNC.  RANSAC classifies 
% Points that support the model as inliers and those that do not as outliers.
%
% X typically contains corresponding point data, one column per point pair.
% RANSAC determines the subset of points (inliers) that best fit the model 
% described by the function FUNC and the parameter M.  T is a threshold on
% how well a point fits the estimated, if the fit residual is aboe the
% the threshold the point is considered an outlier.
%
% [M,IN] = RANSAC(FUNC, X, T, OPTIONS) as above but returns the vector IN of
% column indices of X that describe the inlier point set.
%
% [M,IN,RESID] = RANSAC(FUNC, X, T, OPTIONS) as above but returns the final
% residual of applying FUNC to the inlier set.
%
% Options::
% 'maxTrials',N       maximum number of iterations (default 2000)
% 'maxDataTrials',N   maximum number of attempts to select a non-degenerate 
%                     data set (default 100)
%
% Model function::
%
% OUT = FUNC(R) is the function passed to RANSAC and it must accept 
% a single argument R which is a structure:
%
%   R.cmd          the operation to perform which is either (string)
%   R.debug        display what's going on (logical)
%   R.X            data to work on, N point pairs (6xN)
%   R.t            threshold (1x1)
%   R.theta        estimated quantity to test (3x3)
%   R.misc         private data (cell array)
%
% The function return value is also a structure:
%
%   OUT.s          sample size (1x1)
%   OUT.X          conditioned data (2DxN)
%   OUT.misc       private data (cell array)
%   OUT.inliers    list of inliers (1xM)
%   OUT.valid      if data is valid for estimation (logical)
%   OUT.theta      estimated quantity (3x3)
%   OUT.resid      model fit residual (1x1)
%
% The values of R.cmd are:
%  'size'          OUT.s is the minimum number of points required to compute
%                  an estimate to OUT.s
%  'condition'     OUT.X = CONDITION(R.X) condition the point data 
%  'decondition'   OUT.theta = DECONDITION(R.theta) decondition the estimated 
%                  model data
%  'valid'         OUT.valid is true if a set of points is not degenerate,
%                  that is they will produce a model.  This is used to discard
%                  random samples that do not result in useful models.
%  'estimate'      [OUT.theta,OUT.resid] = EST(R.X) returns the best fit model 
%                  and residual for the subset of points R.X.  If this function 
%                  cannot fit a model then OUT.theta = [].  If multiple models
%                  are found OUT.theta is a cell array.
%  'error'         [OUT.inliers,OUT.theta] = ERR(R.theta,R.X,T) evaluates the 
%                  distance from the model(s) R.theta to the points R.X and
%                  returns the best model OUT.theta and the subset of R.X that 
%                  best supports (most inliers) that model.
%
% Notes::
% - For some algorithms (eg. fundamental matrix) it is necessary to condition
%   the data to improve the accuracy of model estimation.  For efficiency
%   the data is conditioned once, and the data transform parameters are kept 
%   in the .misc element.  The inverse conditioning operation is applied to
%   the model to transform the estimate based on conditioned data to a model
%   applicable to the original data.
% - The functions FMATRIX and HOMOG are written so as to be callable from
%   RANSAC, that is, they detect a structure argument.
%
% References::
%  - M.A. Fishler and  R.C. Boles. "Random sample concensus: A paradigm
%    for model fitting with applications to image analysis and automated
%    cartography". Comm. Assoc. Comp, Mach., Vol 24, No 6, pp 381-395, 1981
%  - Richard Hartley and Andrew Zisserman. "Multiple View Geometry in
%    Computer Vision". pp 101-113. Cambridge University Press, 2001
%
% Author::
%  Peter Kovesi
%  School of Computer Science & Software Engineering
%  The University of Western Australia
%  pk at csse uwa edu au    
%  http://www.csse.uwa.edu.au/~pk
%
% See also FMATRIX, HOMOGRAPHY.

function [M, inliers, resid] = ransac(fittingfn, x, t, varargin);

    %useRandomsample = ~exist('randsample');

    opt.maxTrials = 2000;
    opt.maxDataTrials = 100;

    opt = tb_optparse(opt, varargin);
    
    [rows, npts] = size(x);                 
    
    p = 0.99;         % Desired probability of choosing at least one sample
                      % free from outliers

    bestM = NaN;      % Sentinel value allowing detection of solution failure.
    trialcount = 0;
    bestscore =  0;    
    N = 1;            % Dummy initialisation for number of trials.
    
    out = invoke('size', fittingfn, opt.verbose, []);
    s = out.s;


    in.X = x;
    out = invoke('condition', fittingfn, opt.verbose, in);
    x = out.X;
    misc = out.misc;
    


    while N > trialcount
        
        % Select at random s datapoints to form a trial model, M.
        % In selecting these points we have to check that they are not in
        % a degenerate configuration.
        degenerate = 1;
        count = 1;
        while degenerate
            % Generate s random indicies in the range 1..npts
            % (If you do not have the statistics toolbox, or are using Octave,
            % use the function RANDOMSAMPLE from my webpage)
        if 0
	    if useRandomsample
            ind = randomsample(npts, s);
	    else
            ind = randsample(npts, s);
	    end
        else
            ind = randi(npts, 1, s);
        end

            % Test that these points are not a degenerate configuration.

            in.X = x(:,ind);
            out = invoke('valid', fittingfn, opt.verbose, in);
            degenerate = ~out.valid;
            
            if ~degenerate
                % Fit model to this random selection of data points.
                % Note that M may represent a set of models that fit the data in
                % this case M will be a cell array of models

                in.X = x(:,ind);
                out = invoke('estimate', fittingfn, opt.verbose, in);
                M = out.theta;
                
                % Depending on your problem it might be that the only way you
                % can determine whether a data set is degenerate or not is to
                % try to fit a model and see if it succeeds.  If it fails we
                % reset degenerate to true.
                if isempty(M)
                    degenerate = 1;
                end
            end
            
            % Safeguard against being stuck in this loop forever
            count = count + 1;
            if count > opt.maxDataTrials
                warning('Unable to select a nondegenerate data set');
                break
            end
        end
        
        % Once we are out here we should have some kind of model...        
        % Evaluate distances between points and model returning the indices
        % of elements in x that are inliers.  Additionally, if M is a cell
        % array of possible models 'distfn' will return the model that has
        % the most inliers.  After this call M will be a non-cell object
        % representing only one model.

        in.t = t;
        in.theta = M;
        in.X = x;
        out = invoke('error', fittingfn, opt.verbose, in);
        inliers = out.inliers;
        M = out.theta;
        
        % Find the number of inliers to this model.
        ninliers = length(inliers);
        
        if ninliers > bestscore    % Largest set of inliers so far...
            bestscore = ninliers;  % Record data for this model
            bestinliers = inliers;
            bestM = M;
            
            % Update estimate of N, the number of trials to ensure we pick, 
            % with probability p, a data set with no outliers.
            fracinliers =  ninliers/npts;
            pNoOutliers = 1 -  fracinliers^s;
            pNoOutliers = max(eps, pNoOutliers);  % Avoid division by -Inf
            pNoOutliers = min(1-eps, pNoOutliers);% Avoid division by 0.
            N = log(1-p)/log(pNoOutliers);
        end
        
        trialcount = trialcount+1;
        if opt.verbose > 1
            fprintf('trial %d out of %d         \r',trialcount, ceil(N));
        end

        % Safeguard against being stuck in this loop forever
        if trialcount > opt.maxTrials
            warning( ...
            sprintf('ransac reached the maximum number of %d trials',...
                    opt.maxTrials));
            break
        end     
    end
    fprintf('\n');
    
    if ~isnan(bestM)   % We got a solution 
        M = bestM;
        inliers = bestinliers;
    else           
        M = [];
        inliers = [];
        error('ransac was unable to find a useful solution');
    end

    % Now do a final least squares fit on the data points considered to
    % be inliers.
    [M,resid] = feval(fittingfn, x(:,inliers)); % with conditioned data

    % then decondition it
    in.theta = M;
    in.misc = misc;
    out = invoke('decondition', fittingfn, opt.verbose, in);
    M = out.theta;

    if opt.verbose || (nargout == 0)
        fprintf('%d trials\n', trialcount);
        fprintf('%d outliers\n', npts-length(inliers));
        fprintf('%g final residual\n', resid);
    end
end

function out = invoke(cmd, func, verbose, in)
    if isempty(in) || (nargin < 4)
        in = [];
    end
    in.debug = verbose > 1;
    in.cmd = cmd;
    
    if verbose > 2,
        fprintf('---------------------------------------------\n');
        in
    end
    out = feval(func, in);
    
    if verbose > 2,
        out
    end
end
