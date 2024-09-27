function [ funstats] = getprofilefunctionstats( pstats , desfun, varargin)
%GETPROFILEFUNCTIONSTATS Summary of this function goes here
%   Detailed explanation goes here

nEl = numel(pstats.FunctionTable);
desfname = which(desfun);
funstats = [];

if isempty(desfname)
    error(['Function ', desfun, ' not found!']);
end

funtype = '';
if nargin == 3
    funtype = lower(varargin{1});
    
        if ~(strcmp(funtype,'m-function') || strcmp(funtype,'mex-function'))
            error('funtype must be either ''M-function'' or ''MEX-function''!');
        end
end

for iEl = 1:nEl
    curstats = pstats.FunctionTable(iEl);
    
    if (strcmp(curstats.FileName,desfname))
        if strcmpi(curstats.Type,funtype) || isempty(funtype)
            funstats = curstats;
            return
        end
    end
    
end

end

