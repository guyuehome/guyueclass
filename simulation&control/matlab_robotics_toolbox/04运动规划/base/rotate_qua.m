%四元数计算较为方便但理解上不直观
q0=Quaternion(R0);
q1=Quaternion(R1);
q= interp(q0,q1,[0:49]'/49);
tranimate(q)