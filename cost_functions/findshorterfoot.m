function foot = findshorterfoot(base_foot_coords,new_center_coord)
lengths = sqrt(sum((base_foot_coords - new_center_coord).^2,2));

close_foot_coords = base_foot_coords(lengths~=max(lengths),:);

new_foot_coords = new_center_coord*3 - close_foot_coords(1,:) - close_foot_coords(2,:);

foot = new_foot_coords;


end