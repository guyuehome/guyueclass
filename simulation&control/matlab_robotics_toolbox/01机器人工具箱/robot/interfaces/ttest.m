hh = [];
for i=1:200
    e = s.get();
    
    th = [e.motionRoll e.motionPitch e.motionYaw]
    R = rpy2r(th);
    trplot(R)
    hh = [hh; th];
    pause(0.2)
end
