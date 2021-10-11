release = load('RELEASE');
fprintf('- Machine Vision Toolbox for Matlab (release %.1f)\n', release);
tbpath = fileparts(which('blackbody'));
addpath( fullfile(tbpath, 'examples') );
addpath( fullfile(tbpath, 'images') );
addpath( fullfile(tbpath, 'mex') );
% add the contrib code to the path
p = fullfile(rvcpath, 'contrib/vgg');
if exist(p)
    addpath( p );
    disp([' - VGG contributed code (' p ')']);
end
p = fullfile(rvcpath, 'contrib/EPnP');
if exist(p)
    addpath( p );
    disp([' - EPnP contributed code (' p ')']);
end
p = fullfile(rvcpath, ['contrib/vlfeat-0.9.9/toolbox/mex/' mexext]);
if exist(p)
    addpath( p );
    disp([' - VLFeat contributed code (' p ')']);
    
    p = fullfile(rvcpath, 'contrib/sift');
    if exist(p)
        addpath( p );
        disp([' - VLFeat SIFT wrapper (' p ')']);
    end
end

p = fullfile(rvcpath, 'contrib/surf');
if exist(p)
    addpath( p );
    disp([' - OpenSURF contributed code + wrapper (' p ')']);
end
p = fullfile(rvcpath, 'contrib/graphseg');
if exist(p)
    addpath( p );
    disp([' - graphseg contributed code (' p ')']);
end
clear p release tbpath
