elist = data.n_ini.elist;

for i = 1:length(data.x)
    pos = reshape(data.x(i,:),size(data.n_ini.pos));
    manip_data(i,1) = manipulability_cal(elist,pos);
end

figure
plot(manip_data)