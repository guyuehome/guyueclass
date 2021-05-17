function samplePoint = getSamplePoint(field, treeNodes)
[rows, cols] = size(field);
field(treeNodes(:,1)) = 3;
while true
    samplePoint = randi([1,rows*cols]);
    if field(samplePoint) == 1
        break;
    end
end