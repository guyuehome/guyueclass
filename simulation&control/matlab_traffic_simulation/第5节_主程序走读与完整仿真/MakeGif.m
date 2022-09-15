function MakeGif(filename,index)  
    f = getframe(gcf);  
    imind = frame2im(f);  
    [imind,cm] = rgb2ind(imind,256);  
    if index==1  
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.001);
    else  
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.001);
    end  
end  