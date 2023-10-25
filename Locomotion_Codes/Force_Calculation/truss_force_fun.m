function val = truss_force_fun(dx,n,x0,data,alpha)

elist = n.elist;
pos = reshape( x0 + dx, size(n.pos));
mass_node = data.mass_node;
mass_member = data.mass_member;
fixed_node = data.fixed_node{end};

P = truss_force_cal(elist,pos,mass_node,mass_member,fixed_node);
val = alpha*max(max(P)) - (1-alpha)*min(min(P));