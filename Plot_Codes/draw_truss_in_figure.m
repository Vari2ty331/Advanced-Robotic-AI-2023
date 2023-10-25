function draw_truss_in_figure(X,n)

Position = reshape(X, size(n.pos));

hold on

for i = 1:length(n.elist)

    a = n.elist(i,1);
    b = n.elist(i,2);

    Xa = Position(a,1);
    Ya = Position(a,2);
    Za = Position(a,3);
    Xb = Position(b,1);
    Yb = Position(b,2);
    Zb = Position(b,3);

    plot3([Xa Xb],[Ya Yb],[Za Zb],'b','LineWidth',2)

end

hold off