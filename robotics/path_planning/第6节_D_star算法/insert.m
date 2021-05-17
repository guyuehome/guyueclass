function [Nodes,openList] = insert(Nodes,openList,pos,h_new)
Nodes(pos).h = h_new;
if isequal(Nodes(pos).t, 'new')
    Nodes(pos).k = h_new;
else
    Nodes(pos).k = min( Nodes(pos).k, h_new);
end
Nodes(pos).t = 'open';
openList(end+1,:) = [pos,Nodes(pos).k];

