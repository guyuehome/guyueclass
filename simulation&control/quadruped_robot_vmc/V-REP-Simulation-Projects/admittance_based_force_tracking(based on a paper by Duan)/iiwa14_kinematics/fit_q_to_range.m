function [result,modified_q] = fit_q_to_range(qmin,qmax,q)
modified_q = q;
while modified_q<qmin
    modified_q = modified_q+2*pi;
end
while modified_q>qmax
    modified_q = modified_q-2*pi;
end
if modified_q<qmin
    result = false;
else
    result = true;
end
end