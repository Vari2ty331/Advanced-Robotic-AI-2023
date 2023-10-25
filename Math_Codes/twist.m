function qf = twist(qi,p1,p2,theta)
% qi = n.pos(i,:);

if size(qi,1) < size(qi,2)
    qi = qi';
end

if size(p1,1) < size(p1,2)
    p1 = p1';
end

if size(p2,1) < size(p2,2)
    p2 = p2';
end



% transform qi to qf by rotation axis (p1-p2) with theta
% ref: Murray.'A mathmatical introduction to robotic manipulation'.28p,39-42p

w = (p2 - p1)/norm(p2 - p1);
w_hat = skew(w);
v = -cross(w,p1);

R = eye(3) + w_hat*sin(theta) + w_hat^2*(1-cos(theta));
g(1:3,1:3) = R;
g(1:3,4) = (eye(3) - R)*cross(w,v) + w*w'*v*theta;
g(4,1:3) = [0 0 0];
g(4,4) = 1;

qf = g*[qi;1];
qf = qf(1:3);
