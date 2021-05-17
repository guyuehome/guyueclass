function h = calculateH(pos_now, pos_goal, rows, cols)
% 这里的节点到目标点的估计值h计算方式与A*算法定义不同
% A*算法是直接用本节点和目标点的h = 行差 + 列差
% 本系列课程考虑到需求多样性，这里采用另外一种计算方式
% 将估计值定义为h = 最大斜向运动次数 + 横向（纵向）运动次数  

[pos_now_sub(1),pos_now_sub(2)] = ind2sub([rows, cols], pos_now);
[pos_goal_sub(1),pos_goal_sub(2)] = ind2sub([rows, cols], pos_goal);
diff_row = abs(pos_now_sub(1) - pos_goal_sub(1));
diff_col = abs(pos_now_sub(2) - pos_goal_sub(2));
diff_min = min(diff_row,diff_col);
diff_max = max(diff_row,diff_col);
h = sqrt(2)*diff_min + (diff_max - diff_min);
end
