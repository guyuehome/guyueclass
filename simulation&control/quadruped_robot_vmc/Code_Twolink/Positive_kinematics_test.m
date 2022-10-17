PI=3.1415;
%单腿结构参数
h=0.15;
hu=0.35;
hl=0.382;
%身体结构参数
b=0.4;
l=0.8;
w=0.7;
height=0.732;
centroid = [0 0 0.732]; %质心

%%
%右后腿正运动学测试
gamma=0/180*PI;
alfa=30/180*PI;
beta=-50/180*PI;
rb_rot1  = [-0.4 -0.2 0.732];
rb_rot2  = [rb_rot1(1) rb_rot1(2)-h*cos(gamma) rb_rot1(3)+h*sin(gamma)];
rb_rot3  = [rb_rot1(1)-hu*sin(alfa)       rb_rot2(2)-hu*cos(alfa)*sin(gamma)  rb_rot2(3)-hu*cos(alfa)*cos(gamma)];
rb_dummy = [rb_rot3(1)-hl*sin(beta+alfa)    rb_rot3(2)-hl*cos(beta+alfa)*sin(gamma)  rb_rot3(3)-hl*cos(beta+alfa)*cos(gamma)];

% [a,b,c] =Positive_kinematics(gamma,alfa,beta);
% rb_dummy=[rb_rot1(1)+a rb_rot1(2)+b rb_rot1(3)+c];

%左后腿正运动学测试
gamma=0/180*PI;
alfa=30/180*PI;
beta=-50/180*PI;
lb_rot1  = [-0.4 0.2 0.732];
lb_rot2  = [lb_rot1(1) lb_rot1(2)+h*cos(gamma) lb_rot1(3)+h*sin(gamma)];
lb_rot3  = [lb_rot1(1)-hu*sin(alfa)       lb_rot2(2)+hu*cos(alfa)*sin(gamma)  lb_rot2(3)-hu*cos(alfa)*cos(gamma)];  
lb_dummy = [lb_rot3(1)-hl*sin(beta+alfa)    lb_rot3(2)+hl*cos(beta+alfa)*sin(gamma)  lb_rot3(3)-hl*cos(beta+alfa)*cos(gamma)];

%右前腿正运动学测试
gamma=0/180*PI;
alfa=30/180*PI;
beta=-50/180*PI;
rf_rot1  = [0.4 -0.2 0.732];
rf_rot2  = [rf_rot1(1) rf_rot1(2)-h*cos(gamma) rf_rot1(3)+h*sin(gamma)];
rf_rot3  = [rf_rot1(1)-hu*sin(alfa)       rf_rot2(2)-hu*cos(alfa)*sin(gamma)  rf_rot2(3)-hu*cos(alfa)*cos(gamma)];
rf_dummy = [rf_rot3(1)-hl*sin(beta+alfa)    rf_rot3(2)-hl*cos(beta+alfa)*sin(gamma)  rf_rot3(3)-hl*cos(beta+alfa)*cos(gamma)];

%左前腿正运动学测试
gamma=0/180*PI;
alfa=30/180*PI;
beta=-50/180*PI;
lf_rot1  = [0.4 0.2 0.732];
lf_rot2  = [lf_rot1(1) lf_rot1(2)+h*cos(gamma) lf_rot1(3)+h*sin(gamma)];
lf_rot3  = [lf_rot1(1)-hu*sin(alfa)       lf_rot2(2)+hu*cos(alfa)*sin(gamma)  lf_rot2(3)-hu*cos(alfa)*cos(gamma)];  
lf_dummy = [lf_rot3(1)-hl*sin(beta+alfa)    lf_rot3(2)+hl*cos(beta+alfa)*sin(gamma)  lf_rot3(3)-hl*cos(beta+alfa)*cos(gamma)];


body_point=[ rb_rot1;
             rf_rot1; 
             lf_rot1;
             lb_rot1;
             rb_rot1   ];

rb_point=[   rb_rot1;
             rb_rot2;
             rb_rot3;
             rb_dummy  ];

lb_point=[   lb_rot1;
             lb_rot2;
             lb_rot3;
             lb_dummy  ];

rf_point=[   rf_rot1;
             rf_rot2;
             rf_rot3;
             rf_dummy  ];

lf_point=[   lf_rot1;
             lf_rot2;
             lf_rot3;
             lf_dummy  ];


pos=[
     body_point;
     rb_point;
     lb_point;
     rf_point;
     lf_point;];
x=pos(:,1);y=pos(:,2);z=pos(:,3);
scatter3(x,y,z,100,'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);hold on;
scatter3(centroid(1),centroid(2),centroid(3),250,'filled','ColorVariable','Diastolic');
axis([-1 1 -1 1 0 2]);
%画身体
body_x=body_point(:,1);body_y=body_point(:,2);body_z=body_point(:,3);
line(body_x,body_y,body_z,'Color','red','LineStyle','-','LineWidth',5);hold on;
%画右后腿
rb_x=rb_point(:,1);rb_y=rb_point(:,2);rb_z=rb_point(:,3);
line(rb_x,rb_y,rb_z,'Color',[0.9290 0.6940 0.1250],'LineStyle','-','LineWidth',5);hold on;
%画左后腿
lb_x=lb_point(:,1);lb_y=lb_point(:,2);lb_z=lb_point(:,3);
line(lb_x,lb_y,lb_z,'Color',[0.9290 0.6940 0.1250],'LineStyle','-','LineWidth',5);hold on;
%画右前腿
rf_x=rf_point(:,1);rf_y=rf_point(:,2);rf_z=rb_point(:,3);
line(rf_x,rf_y,rf_z,'Color',[0.9290 0.6940 0.1250],'LineStyle','-','LineWidth',5);hold on;
%画左后腿
lf_x=lf_point(:,1);lf_y=lf_point(:,2);lf_z=lb_point(:,3);
line(lf_x,lf_y,lf_z,'Color',[0.9290 0.6940 0.1250],'LineStyle','-','LineWidth',5);hold on;
%%
%雅可比矩阵求解，针对右后腿
syms gamma alfa beta hu hl h
x=-hu*sin(alfa)-hl*sin(beta+alfa);
y=-h*cos(gamma)-hu*cos(alfa)*sin(gamma)-hl*cos(beta+alfa)*sin(gamma);
z=h*sin(gamma)-hu*cos(alfa)*cos(gamma)-hl*cos(beta+alfa)*cos(gamma);
j=jacobian([x y z],[gamma alfa beta]);
%二自由度
syms th1 th2 hu hl 
x= hu*sin(th1) + hl*sin(th2+th1);
z= hu*cos(th1) + hl*cos(th2+th1);
J=jacobian([x z],[th1 th2]);