function h = calculateH(pos_now, pos_start, rows, cols)
[pos_now_sub(1),pos_now_sub(2)] = ind2sub([rows, cols], pos_now);
[pos_goal_sub(1),pos_goal_sub(2)] = ind2sub([rows, cols], pos_start);
diff_row = abs(pos_now_sub(1) - pos_goal_sub(1));
diff_col = abs(pos_now_sub(2) - pos_goal_sub(2));
diff_min = min(diff_row,diff_col);
diff_max = max(diff_row,diff_col);
h = sqrt(2)*diff_min + (diff_max - diff_min);
end
