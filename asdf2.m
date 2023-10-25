hold on
axis equal
axis ([-5 5 -5 5])
tree = tree;
plot(tree.T(1).n(:,1),tree.T(1).n(:,2),'.',MarkerSize=20)
for i = 2:length(tree.T)
plot_2D_dots(tree.T(i).n);
pause(0.01)
end