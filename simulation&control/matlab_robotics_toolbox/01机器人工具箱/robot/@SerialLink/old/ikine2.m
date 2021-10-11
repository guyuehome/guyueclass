
function [qt,histout] = ikine2(robot, tr, varargin)
    %  set default parameters for solution
    opt.ilimit = 1000;
    opt.tol = 1e-6;
    opt.alpha = 1;
    opt.plot = false;
    opt.pinv = true;
    opt.varstep = false;
    
    tau = 1e-2;
    v = 2;

    [opt,args] = tb_optparse(opt, varargin);

    n = robot.n;

    % robot.ikine(tr, q)
    if ~isempty(args)
        q = args{1};
        if isempty(q)
            q = zeros(1, n);
        else
            q = q(:)';
        end
    else
        q = zeros(1, n);
    end

    % robot.ikine(tr, q, m)
    if length(args) > 1
        m = args{2};
        m = m(:);
        if numel(m) ~= 6
            error('RTB:ikine:badarg', 'Mask matrix should have 6 elements');
        end
        if numel(find(m)) > robot.n 
            error('RTB:ikine:badarg', 'Number of robot DOF must be >= the same number of 1s in the mask matrix')
        end
    else
        if n < 6
            error('RTB:ikine:badarg', 'For a manipulator with fewer than 6DOF a mask matrix argument must be specified');
        end
        m = ones(6, 1);
    end
    % make this a logical array so we can index with it
    m = logical(m);

    npoints = size(tr,3);    % number of points
    qt = zeros(npoints, n);  % preallocate space for results
    tcount = 0;              % total iteration count

    if ~ishomog(tr)
        error('RTB:ikine:badarg', 'T is not a homog xform');
    end

    J0 = jacob0(robot, q);
    J0 = J0(m, :);
    if cond(J0) > 100
        warning('RTB:ikine:singular', 'Initial joint angles results in near-singular configuration, this may slow convergence');
    end

    history = [];
    failed = false;
    e = zeros(6,1);
    
    revolutes = [robot.links.sigma] == 0;
    
    for i=1:npoints       % for each transform on trajectory
        T = tr(:,:,i);

        nm = Inf;
        % initialize state for the ikine loop
        eprev = Inf;
        save.e = [Inf Inf Inf Inf Inf Inf];
        save.q = [];
        count = 0;

        while true      % iterate the solution
            % update the count and test against iteration limit
            count = count + 1;
            if count > opt.ilimit
                warning('ikine: iteration limit %d exceeded (row %d), final err %f', ...
                    opt.ilimit, i, nm);
                failed = true;
                %q = NaN*ones(1,n);
                break
            end
            
            % compute the error
            [en,e] = errfun(robot, T, q, m);
            
            if en <= opt.tol
                break
            end
            
            % compute the Jacobian
            J = jacob0(robot, q);
            
            % compute change in joint angles to reduce the error,
            % based on the square sub-Jacobian
            
            J = J(m,:);
                
            JJ = J'*J;
            lambda = tau *diag(JJ);
            
                
            while true    % the Levenberg-Marquadt iteration
                
                dq1 = inv(JJ + diag(lambda))*J' * e(m)';
                en1 = errfun(robot, T, q-dq1', m);
                
%                 dq2 = inv(JJ + diag(lambda/v))*J' * e(m)';
%                 en2 = errfun(robot, T, q-dq2', m);

                    if en1 > en
                        disp('increase lambda');
                        lambda = lambda * v;
                    else
                        q = q - dq1';
                        break;   % dq1 is good
                    end                    
                
                [en en1]

%                 if en2 > en
%                     if en1 > en
%                         disp('increase lambda');
%                         lambda = lambda * v;
%                     else
%                         break;   % dq1 is good
%                     end
%                 else
%                    disp('decrease lambda');
% 
%                   lambda = lambda / v;
%                 end
            end
            

            disp(' done');

            % update the estimated solution
            %q = q + dq1';
            
            % wrap angles for revolute joints
            k = (q > pi) & revolutes;
            q(k) = q(k) - 2*pi;
            
            k = (q < -pi) & revolutes;
            q(k) = q(k) + 2*pi;
            

            if norm(e) > 1.5*norm(eprev)
                warning('RTB:ikine:diverged', 'solution diverging, try reducing alpha');
            end
            eprev = e;


        end  % end ikine solution for tr(:,:,i)
        qt(i,:) = q';
        tcount = tcount + count;
        if opt.verbose
            fprintf('%d iterations\n', count);
        end
    end
    
    if opt.verbose && npoints > 1
        fprintf('TOTAL %d iterations\n', tcount);
    end


end

function [nm,e] = errfun(robot, T, q, m)
    Tq = robot.fkine(q');
    
    e(1:3) = transl(T - Tq);
    Rq = t2r(Tq);
    [th,n] = tr2angvec(t2r(T)*Rq');
    e(4:6) = th*n;
    
    nm = norm(e(m));
    
end
