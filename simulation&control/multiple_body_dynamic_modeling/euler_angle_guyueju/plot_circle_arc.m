function plot_circle_arc(cir_cen,start_point,end_point,co)
% this function can plot arc
% the cir_cen is the center of circle
% the start_point is the start point of arc
% the end_point is the end point of arc
N       = 20;
r       = norm(start_point-cir_cen);
dpoint  = (end_point-start_point)/N;
dir_    = [start_point(1,1):dpoint(1,1):end_point(1,1);
            start_point(2,1):dpoint(2,1):end_point(2,1);
            start_point(3,1):dpoint(3,1):end_point(3,1);];
dir_no  = vecnorm(dir_);
arc_po  = r.*dir_./dir_no;
if size(dir_,1)==2
    plot(arc_po(1,:),arc_po(2,:),'Color',co,'LineWidth',2,'LineStyle','--')
else
    plot3(arc_po(1,:),arc_po(2,:),arc_po(3,:),'Color',co,'LineWidth',2,'LineStyle','--')
end
end


