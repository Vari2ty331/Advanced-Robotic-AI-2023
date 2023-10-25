function dist_x_fin = distance_from_x_fin(x,dx,x_fin)

x_new = x + dx;

dist_x_fin = norm(x_new - x_fin);
