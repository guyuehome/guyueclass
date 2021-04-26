sim_out=sim('sim_test','SimulationMode','Normal','stoptime','30');

param_struct=struct('SimulationMode','Normal','stoptime','30');
sim_out=sim('sim_test',param_struct);

%%
param_struct1.SaveState      = 'on';
param_struct1.StateSaveName  = 'xout1';
param_struct1.SaveOutput     = 'on';
param_struct1.OutputSaveName = 'yout1';
param_struct1.SolverType     = 'Fixed-step';
param_struct1.Solver         = 'FixedStepDiscrete';
param_struct1.FixedStep      = '0.01';
sim_out1 = sim('sim_test',param_struct1);
param_struct2 = param_struct1;
param_struct2.FixedStep      = '0.2';
param_struct2.OutputSaveName ='yout2';
sim_out2 = sim('sim_test',param_struct2);
t1 = get(sim_out1, 'tout');
t2 = get(sim_out2, 'tout');
y1 = get(sim_out1, 'yout');
y2 = get(sim_out2, 'yout');
figure;
subplot(211);
plot(t1,y1(:,2));
xlabel('time(s)');
ylabel('yout1');
subplot(212);
plot(t2,y2(:,2));
xlabel('time(s)');
ylabel('yout2');

%%
set_param('sim_test','SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','0.1');
set_param('sim_test', 'SimulationCommand', 'start');
set_param('sim_test', 'SimulationCommand', 'pause');
set_param('sim_test', 'SimulationCommand', 'step');
pause(0.2);
t = get_param('sim_test', 'SimulationTime');  % get current simulation time
while t~=0
    t = get_param('sim_test', 'SimulationTime');  % get current simulation time
    if t < 3
        set_param('sim_test/Gain', 'Gain','3');
    elseif t < 8
        set_param('sim_test/Gain', 'Gain','1.5');
    else
        set_param('sim_test/Gain', 'Gain','-0.3');
    end
    set_param('sim_test', 'SimulationCommand', 'step');
    pause(0.2);
end
set_param('sim_test', 'SimulationCommand', 'stop');
