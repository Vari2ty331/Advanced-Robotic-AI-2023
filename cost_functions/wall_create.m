function wall = wall_create(base,wall_start,wall_end,height)
% Ground_base = [             -2.    -2.      0;
%                             -2,     5,      0;
%                              5,    -2,      0;
%                              5,     5,      0;];
% ref = [                     0.6,    -2,     0;
%                             0.62,   -2,     0;
%                             0.6,    5,      0;
%                             0.62,   5,      0;
%                             0.6,    1,      0;
%                             0.62,   1,      0;
%                             0.61,   0.9,    0;
%                             0.6,    3,      0;
%                             0.62,   3,      0;
%                             0.61,   3.1,    0;
%                             0.61,   1,      1.5;
%                             0.61,   3,      1.5;];
% wall_start = [0.6 1];
% wall_end = [0.6 3];
% height = 1.5;

offset = 0.01;
base_x_min = min(base(:,1));
base_x_max = max(base(:,1));
base_y_min = min(base(:,2));
base_y_max = max(base(:,2));

if(wall_start(1)==wall_end(1))
    wall = [wall_start(1)           , base_y_min    , 0;
            wall_start(1)+offset*2  , base_y_min    , 0;
            wall_start(1)           , base_y_max    , 0;
            wall_start(1)+offset*2  , base_y_max    , 0;
            wall_start(1)           , wall_start(2)    , 0;
            wall_start(1)+offset*2  , wall_start(2)    , 0;
            wall_start(1)+offset    , wall_start(2)-offset*10    , 0;
            wall_start(1)           , wall_end(2)    , 0;
            wall_start(1)+offset*2  , wall_end(2)    , 0;
            wall_start(1)+offset    , wall_end(2)+offset*10      , 0;
            wall_start(1)+offset    , wall_start(2)    , height;
            wall_start(1)+offset    , wall_end(2)     , height;];
elseif(wall_start(2)==wall_end(2))
    wall_start = fliplr(wall_start);
    wall_end   = fliplr(wall_end);
    wall = [wall_start(1)           , base_x_min    , 0;
            wall_start(1)+offset*2  , base_x_min    , 0;
            wall_start(1)           , base_x_max    , 0;
            wall_start(1)+offset*2  , base_x_max    , 0;
            wall_start(1)           , wall_start(2)    , 0;
            wall_start(1)+offset*2  , wall_start(2)    , 0;
            wall_start(1)+offset    , wall_start(2)-offset*10    , 0;
            wall_start(1)           , wall_end(2)    , 0;
            wall_start(1)+offset*2  , wall_end(2)    , 0;
            wall_start(1)+offset    , wall_end(2)+offset*10      , 0;
            wall_start(1)+offset    , wall_start(2)    , height;
            wall_start(1)+offset    , wall_end(2)     , height;];
    temp = wall;
    temp(:,2) = wall(:,1);
    temp(:,1) = wall(:,2);
    wall = temp;
else
    wall = [];
end

end