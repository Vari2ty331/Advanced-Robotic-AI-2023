function H_Rotated = Rotating_Line(H, P1, P2, theta)
% All vectors are column vectors
% H : Point that will be rotated.
% H_Rotated : Rotated Point H
% P1 : One point of rotation line.
% P2 : The other point of rotation line.
% theta: rotation angle

    Rot_Axis = P2 - P1;
    x0 = P1(1);
    y0 = P1(2);
    z0 = P1(3);

    Rot_Axis = Rot_Axis / norm(Rot_Axis);

    H_zero = H + [-x0; -y0; -z0;];

    H_zero_Q = quaternion(0, H_zero(1), H_zero(2), H_zero(3));

    Rotation_Q = quaternion(cos(theta/2), Rot_Axis(1)*sin(theta/2), Rot_Axis(2)*sin(theta/2), Rot_Axis(3)*sin(theta/2));

    H_Rot_Q = Rotation_Q * H_zero_Q * conj(Rotation_Q);
    [~, H_Rot(1), H_Rot(2), H_Rot(3)] = parts(H_Rot_Q);

    H_Rotated = H_Rot.' + [x0; y0; z0;];