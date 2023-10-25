function bool = checkFootMadePossibleWithMargin(base_foot_coords,new_center_coord)

L_nom = 1.25;
step = 0.15;

lengths = sqrt(sum((base_foot_coords - new_center_coord).^2,2));

close_foot_coords = base_foot_coords(lengths~=max(lengths),:);

if sum(lengths~=max(lengths)) < 2
bool = false;
else

new_foot_coords = new_center_coord*3 - close_foot_coords(1,:) - close_foot_coords(2,:);

if(sqrt(sum((close_foot_coords(1,:) - new_foot_coords).^2,2))> L_nom-step && sqrt(sum((close_foot_coords(1,:) - new_foot_coords).^2,2))<L_nom+step && sqrt(sum((close_foot_coords(2,:) - new_foot_coords).^2,2))>L_nom-step && sqrt(sum((close_foot_coords(2,:) - new_foot_coords).^2,2))<L_nom+step)

    bool = true;

else

    bool = false;

end

end

end