function [outSets,minMinPath,minMinPathIdx,loopTime] = rapidPolygonPathFunction(polygon,startIndex,endIndex)

% polygon = createRandomPolygon(n);

% startIndex = randIndex(:,1)';
% endIndex = randIndex(:,2)';



%%

% for i = 1:length(polygon)
%     if  i ~= length(polygon)
%         edge(i).index = [i,i+1];
%         edge(i).coords = [polygon(i,:);polygon(i+1,:)];
%     else
%         edge(i).index = [1,i];
%         edge(i).coords = [polygon(1,:);polygon(i,:)];
%     end
%     if ismember(edge(i).index,startIndex)
%         startEdge = i;
%     elseif ismember(edge(i).index,endIndex)
%         endEdge = i;
%     end
% end
%
% startEndVector = edge(endEdge).coords - edge(startEdge).coords;
startCenter = sum(polygon(startIndex,:))/2;
endCenter = sum(polygon(endIndex,:))/2;
startEndVector = endCenter - startCenter;
if any([all(ismember(startIndex,[1 length(polygon)])),all(ismember(endIndex,[1 length(polygon)]))])

    if all(ismember(endIndex,[1 length(polygon)]))
        cwStart = min(startIndex);
        ccwStart = max(startIndex);
        ccwEnd = 1;
        cwEnd = length(polygon);
    end
    if all(ismember(startIndex,[1 length(polygon)]))
        ccwStart = 1;
        cwStart = length(polygon);
        cwEnd = max(endIndex);
        ccwEnd = min(endIndex);
    end

else
    cwStart = min(startIndex);
    ccwStart = max(startIndex);
    cwEnd = max(endIndex);
    ccwEnd = min(endIndex);
end

ccwList = circshift(1:length(polygon),1-ccwStart);
cwList = circshift(length(polygon):-1:1,cwStart);

ccwList(find(ccwList==ccwEnd)+1:end) = [];
cwList(find(cwList==cwEnd)+1:end) = [];




endFlag = 0;
i = 2;
j = 2;
k = 1;
ccwBase = ccwStart;
cwBase = cwStart;
base = [ccwBase cwBase];


time = tic;

while(endFlag ~=1)
    if i >= length(ccwList) && j >= length(cwList)
        endFlag = 1;
    end
    % compareVertex1 = polygon(ccwList(i),:);
    % compareVertex2 = polygon(cwList(j),:);
    if ~(i >= length(ccwList) || j >= length(cwList))
        compareCenter2 = sum([polygon(base,:); polygon(cwList(j),:)])/3;
        compareCenter1 = sum([polygon(base,:); polygon(ccwList(i),:)])/3;
        dist1 = norm(cross([compareCenter1 - startCenter],[startEndVector]))/norm(startEndVector);
        dist2 = norm(cross([compareCenter2 - startCenter],[startEndVector]))/norm(startEndVector);
        if dist1>dist2
            set.triangles(k).coords = [polygon(base,:);polygon(cwList(j),:)];
            set.triangles(k).center = sum(set.triangles(k).coords)/3;
            cwBase = cwList(j);
            base = [ccwBase cwBase];
            if j ~= length(cwList)
                j = j + 1;
            else
                j = j;
                if i ~= length(ccwList)
                    i = i + 1;
                    ccwBase = ccwList(i);
                end
            end
        else
            set.triangles(k).coords = [polygon(base,:);polygon(ccwList(i),:)];
            set.triangles(k).center = sum(set.triangles(k).coords)/3;
            ccwBase = ccwList(i);
            base = [ccwBase cwBase];
            if i ~= length(ccwList)
                i = i + 1;
            else
                i = i;
                if j ~= length(cwList)
                    j = j + 1;
                    cwBase = cwList(j);
                end
            end

        end
    else
        if i >= length(ccwList) && j <= length(cwList)
            set.triangles(k).coords = [polygon(base,:);polygon(cwList(j),:)];
            set.triangles(k).center = sum(set.triangles(k).coords)/3;
            cwBase = cwList(j);
            base = [ccwBase cwBase];
            lastVertex = polygon(ccwList(i-1),:);
            j = j + 1;
        elseif j >= length(cwList) && i <= length(ccwList)
            set.triangles(k).coords = [polygon(base,:);polygon(ccwList(i),:)];
            set.triangles(k).center = sum(set.triangles(k).coords)/3;
            ccwBase = ccwList(i);
            base = [ccwBase cwBase];
            lastVertex = polygon(cwList(j-1),:);
            i = i + 1;
        end
    end
    k = k + 1;

end

if length(set.triangles) < (length(polygon) - 2)
    set.triangles(end+1).coords = [polygon(endIndex,:);lastVertex];
    set.triangles(end).center = sum(set.triangles(end).coords)/3;
elseif length(set.triangles) > (length(polygon) - 2)
    set.triangles(end) = [];
end

loopTime = toc(time);

outSets = set;
minMinPathIdx = 1:length(set.triangles);
minMinPath = getDistance(vertcat(set.triangles.center));

end

