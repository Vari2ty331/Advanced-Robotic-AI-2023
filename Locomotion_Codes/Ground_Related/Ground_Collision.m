function [Ground_Collision, Ground_Distance] = Ground_Collision(Ground, Point, MPGeneration)
    if nargin <= 2
        MPGeneration = 0;
    end

    if MPGeneration == 1
        rad = 0.001;
    else
        rad = 0.1;
    end
    Point_Mesh = collisionSphere(rad);
    Point_Mesh.Pose = trvec2tform(Point);

    Ground_Number = size(Ground);
    Ground_Collision = zeros(size(Ground));
    Ground_Distance = zeros(size(Ground));

    for index = 1 : Ground_Number(2)
        [Ground_Collision(1, index), Ground_Distance(index)] = checkCollision(Ground(index), Point_Mesh);
    end