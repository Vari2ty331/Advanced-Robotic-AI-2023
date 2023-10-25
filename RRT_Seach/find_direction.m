% close all
clear
load('truss_ex.mat')
ch = 1;
truss = build_truss(example_truss(ch));
% truss = build_truss(octahedron);

xx = reshape(truss_ex{ch}.pos, [3*length(truss.n),1]);

addvec = 0.05 * [2,-2,1]';
idx = 10;
truss.n(idx).pos  = truss.n(idx).pos +addvec;
xx(3*idx-2:3*idx) = xx(3*idx-2:3*idx) + addvec;
% idx 10 only

% addvec = 0.05 * [2,-2,1]';

% addvec7 = 0.1 * [0, -4, 0]';
% idx = 7;
% truss.n(idx).pos  = truss.n(idx).pos +addvec7;
% xx(3*idx-2:3*idx) = xx(3*idx-2:3*idx) + addvec7;
% idx = 10;
% truss.n(idx).pos  = truss.n(idx).pos +addvec7;
% xx(3*idx-2:3*idx) = xx(3*idx-2:3*idx) + addvec7;
% 
% addvec5 = 0.1 * [0, 0, 4]';
% idx = 5;
% truss.n(idx).pos  = truss.n(idx).pos +addvec5;
% xx(3*idx-2:3*idx) = xx(3*idx-2:3*idx) + addvec5;
% 
% addvec10 = 0.1 * [1, -2, -2]';
% idx = 10;
% truss.n(idx).pos  = truss.n(idx).pos +addvec10;
% xx(3*idx-2:3*idx) = xx(3*idx-2:3*idx) + addvec10;

% final_config = xx;
% save('final_config.mat','final_config')
% initial_config = xx;
% save('initial_config.mat','initial_config')


check_constraints(truss, xx)


tmp_truss.m = truss.m;
tmp_truss.n = truss.n;

tmp_truss.h = plot_3Dtruss(tmp_truss);
xlabel('x')
ylabel('y')
zlabel('z')
