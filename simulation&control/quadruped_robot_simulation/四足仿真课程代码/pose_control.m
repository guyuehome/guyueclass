function [rb_x,rb_y,rb_z,rf_x,rf_y,rf_z,lb_x,lb_y,lb_z,lf_x,lf_y,lf_z] = pose_control(row,pitch,yaw,pos_x,pos_y,pos_z)
b=0.4;
l=0.8;
w=0.7;
h=0.732;
R=row*pi/180   ;
P=pitch*pi/180 ;
Y=yaw*pi/180   ;
pos=[pos_x,pos_y,pos_z]';
rotx=([[ 1,        0,         0      ]
       [ 0,        cos(R),   -sin(R) ]
       [ 0,        sin(R),    cos(R) ]]);
roty=([[ cos(P),   0,        -sin(P) ]
       [ 0,        1,         0      ]
       [ sin(P),   0,         cos(P) ]]);
rotz=([[ cos(Y),  -sin(Y),    0      ]
       [ sin(Y),   cos(Y),    0      ]
       [ 0,        0,         1      ]]);
rot_mat = rotx * roty * rotz;
%结构参数
  body_struct = ([[ l / 2,  -b / 2,  h]     
                 [  l / 2, b / 2,   h]     
                 [ -l / 2,  b / 2,    h]     
                 [ -l / 2, -b / 2,   h]])'; 
             
 footpoint_struct = ([[ l/2,   -w/2,  0]
                     [  l/2,   w/2,  0]
                     [  -l/2,    w/2,  0]
                     [  -l/2,    -w/2,  0]])';
leg_pose = zeros(3, 4);
for i= 1:4
    leg_pose(:,i) =  pos + rot_mat * body_struct(:, i) - footpoint_struct(:, i);
end

 rb_x = (leg_pose(1, 1));
 rb_y = -(leg_pose(2, 1));
 rb_z = -(leg_pose(3, 1));
 rf_x = (leg_pose(1, 2));
 rf_y = (leg_pose(2, 2));
 rf_z = -(leg_pose(3, 2));
 lb_x = (leg_pose(1, 3));
 lb_y = (leg_pose(2, 3));
 lb_z = -(leg_pose(3, 3));
 lf_x = (leg_pose(1, 4));
 lf_y = -(leg_pose(2, 4));
 lf_z = -(leg_pose(3, 4));
end

