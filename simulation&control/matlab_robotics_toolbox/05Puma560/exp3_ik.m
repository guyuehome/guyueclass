qn=[0 0.7854 3.1416 0 0.7854 0] ;
T = p560.fkine(qn); 
figure(1)
p560.plot(qn)
qi = p560.ikine6s(T); 
qi2=p560.ikine6s(T,'ru')
q3= p560.ikine(T)
%ikunc inverse kinematics using optimisation
%ikcon inverse kinematics using optimisation with joint limits
figure(2)
p560.plot(q3)
%figure(3)
%q4=p560.ikunc(T)
%p560.plot(q4)
%figure(4)
%q5=p560.ikcon(T)
%p560.plot(q5)


% 左手或右手 'l','r’ 
% 肘部在上或在下 'u','d’ 
% 手腕翻转或不翻转 'f','n'
