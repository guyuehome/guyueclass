function initial_plot_set_guyueju
global robotarm
robotarm.F          = [1,2,3,4;1,5,8,4;1,5,6,2;5,6,7,8;2,6,7,3;3,4,8,7];
robotarm.V0_arm1    = [-robotarm.le1/2,-robotarm.wi1/2,-robotarm.th1/2;-robotarm.le1/2,robotarm.wi1/2,-robotarm.th1/2;
                        -robotarm.le1/2,robotarm.wi1/2,robotarm.th1/2;-robotarm.le1/2,-robotarm.wi1/2,robotarm.th1/2;
                        robotarm.le1/2,-robotarm.wi1/2,-robotarm.th1/2;robotarm.le1/2,robotarm.wi1/2,-robotarm.th1/2;
                        robotarm.le1/2,robotarm.wi1/2,robotarm.th1/2;robotarm.le1/2,-robotarm.wi1/2,robotarm.th1/2;];
robotarm.V0_arm2    = [-robotarm.le2/2,-robotarm.wi2/2,-robotarm.th2/2;-robotarm.le2/2,robotarm.wi2/2,-robotarm.th2/2;
                        -robotarm.le2/2,robotarm.wi2/2,robotarm.th2/2;-robotarm.le2/2,-robotarm.wi2/2,robotarm.th2/2;
                        robotarm.le2/2,-robotarm.wi2/2,-robotarm.th2/2;robotarm.le2/2,robotarm.wi2/2,-robotarm.th2/2;
                        robotarm.le2/2,robotarm.wi2/2,robotarm.th2/2;robotarm.le2/2,-robotarm.wi2/2,robotarm.th2/2;];
robotarm.V0_arm3    = [-robotarm.le3/2,-robotarm.wi3/2,-robotarm.th3/2;-robotarm.le3/2,robotarm.wi3/2,-robotarm.th3/2;
                        -robotarm.le3/2,robotarm.wi3/2,robotarm.th3/2;-robotarm.le3/2,-robotarm.wi3/2,robotarm.th3/2;
                        robotarm.le3/2,-robotarm.wi3/2,-robotarm.th3/2;robotarm.le3/2,robotarm.wi3/2,-robotarm.th3/2;
                        robotarm.le3/2,robotarm.wi3/2,robotarm.th3/2;robotarm.le3/2,-robotarm.wi3/2,robotarm.th3/2;];
robotarm.V0_arm4    = [-robotarm.le4/2,-robotarm.wi4/2,-robotarm.th4/2;-robotarm.le4/2,robotarm.wi4/2,-robotarm.th4/2;
                        -robotarm.le4/2,robotarm.wi4/2,robotarm.th4/2;-robotarm.le4/2,-robotarm.wi4/2,robotarm.th4/2;
                        robotarm.le4/2,-robotarm.wi4/2,-robotarm.th4/2;robotarm.le4/2,robotarm.wi4/2,-robotarm.th4/2;
                        robotarm.le4/2,robotarm.wi4/2,robotarm.th4/2;robotarm.le4/2,-robotarm.wi4/2,robotarm.th4/2;];
robotarm.V0_arm5    = [-robotarm.le5/2,-robotarm.wi5/2,-robotarm.th5/2;-robotarm.le5/2,robotarm.wi5/2,-robotarm.th5/2;
                        -robotarm.le5/2,robotarm.wi5/2,robotarm.th5/2;-robotarm.le5/2,-robotarm.wi5/2,robotarm.th5/2;
                        robotarm.le5/2,-robotarm.wi5/2,-robotarm.th5/2;robotarm.le5/2,robotarm.wi5/2,-robotarm.th5/2;
                        robotarm.le5/2,robotarm.wi5/2,robotarm.th5/2;robotarm.le5/2,-robotarm.wi5/2,robotarm.th5/2;];
robotarm.V0_arm6    = [-robotarm.le6/2,-robotarm.wi6/2,-robotarm.th6/2;-robotarm.le6/2,robotarm.wi6/2,-robotarm.th6/2;
                        -robotarm.le6/2,robotarm.wi6/2,robotarm.th6/2;-robotarm.le6/2,-robotarm.wi6/2,robotarm.th6/2;
                        robotarm.le6/2,-robotarm.wi6/2,-robotarm.th6/2;robotarm.le6/2,robotarm.wi6/2,-robotarm.th6/2;
                        robotarm.le6/2,robotarm.wi6/2,robotarm.th6/2;robotarm.le6/2,-robotarm.wi6/2,robotarm.th6/2;];
end