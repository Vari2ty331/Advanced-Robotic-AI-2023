function walls = wall_create_not_mesh(n,Ground)

n_case = n;

walls.wall.Start = [];
walls.wall.End = [];

walls.Ground.x_min = min(Ground(1).Vertices(1,:));
walls.Ground.y_min = min(Ground(1).Vertices(2,:));
walls.Ground.x_max = max(Ground(1).Vertices(1,:));
walls.Ground.y_max = max(Ground(1).Vertices(2,:));

switch n_case
    case 0
        walls.wall.Start = [];
        walls.wall.End = [];
    case 1
        walls.wall(1).Start =   [-12 -12 0];
        walls.wall(1).End =     [-12 12 0];
        walls.wall(2).Start =   [-12 12 0];
        walls.wall(2).End =     [12 12 0];
        walls.wall(3).Start =   [12 12 0];
        walls.wall(3).End =     [12 -12 0];
        walls.wall(4).Start =   [12 -12 0];
        walls.wall(4).End =     [-12 -12 0];
    case 2
        walls.wall(1).Start =   [0.0 -3.0 0];
        walls.wall(1).End =     [0.0 3.0 0];
        walls.wall(2).Start =   [0.0 3.0 0];
        walls.wall(2).End =     [5.0 3.0 0];
        walls.wall(3).Start =   [0.0 -3.0 0];
        walls.wall(3).End =     [5.0 -3.0 0];
    case 3
        walls.wall(1).Start =   [0.0 -3.0 0];
        walls.wall(1).End =     [0.0 3.0 0];
        walls.wall(2).Start =   [-5.0 3.0 0];
        walls.wall(2).End =     [5.0 3.0 0];
        walls.wall(3).Start =   [-5.0 -3.0 0];
        walls.wall(3).End =     [5.0 -3.0 0];
    case 4
        walls.wall(1).Start =   [0.0 -3.0 0];
        walls.wall(1).End =     [0.0 3.0 0];
        walls.wall(2).Start =   [0.0 3.0 0];
        walls.wall(2).End =     [4.0 3.0 0];
        walls.wall(3).Start =   [0.0 -3.0 0];
        walls.wall(3).End =     [4.0 -3.0 0];
    case 5
        walls.wall(1).Start =   [-5.0 10 0];
        walls.wall(1).End =     [-5.0 -6 0];
        walls.wall(2).Start =   [0 -10 0];
        walls.wall(2).End =     [0 6 0];
        walls.wall(3).Start =   [5 10 0];
        walls.wall(3).End =     [5 -6 0];
    case 6
        walls.wall(1).Start =   [-4.0 12 0];
        walls.wall(1).End =     [-4.0 -6 0];
        % walls.wall(2).Start =   [0 -12 0];
        % walls.wall(2).End =     [0 6 0];
        walls.wall(3).Start =   [4 -12 0];
        walls.wall(3).End =     [4 6 0];
        walls.wall(4).Start =   [-12 -12 0];
        walls.wall(4).End =     [-12 12 0];
        walls.wall(5).Start =   [-12 12 0];
        walls.wall(5).End =     [12 12 0];
        walls.wall(6).Start =   [12 12 0];
        walls.wall(6).End =     [12 -12 0];
        walls.wall(7).Start =   [12 -12 0];
        walls.wall(7).End =     [-12 -12 0];
    case 7
        walls.wall(1).Start = [-8 -8 0];
        walls.wall(1).End = [8 8 0];
        walls.wall(4).Start =   [-12 -12 0];
        walls.wall(4).End =     [-12 12 0];
        walls.wall(5).Start =   [-12 12 0];
        walls.wall(5).End =     [12 12 0];
        walls.wall(6).Start =   [12 12 0];
        walls.wall(6).End =     [12 -12 0];
        walls.wall(7).Start =   [12 -12 0];
        walls.wall(7).End =     [-12 -12 0];

end




end