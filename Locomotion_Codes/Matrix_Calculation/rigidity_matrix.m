function [ R ] = rigidity_matrix( elist, pos )
%rigidity_matrix makes the rigidity matrix
%   see rigidity theory notes

n1 = elist(:,1);
n2 = elist(:,2);
R = zeros(length(n1), numel(pos)); %rigidity matrix

% for ii = 1:length(m.n1)
%     cols1 = (n1(ii)-1)*3 + (1:3);
%     cols2 = (n2(ii)-1)*3 + (1:3);
%     p1 = pos(:,n1(ii));
%     p2 = pos(:,n2(ii));
%     R(ii, cols1) = (p1 - p2)';
%     R(ii, cols2) = (p2 - p1)';
% end

% This does the same as above, just jammed together for speed
for ii = 1:length(n1)
    R(ii, (n1(ii)-1)*3 + (1:3)) = (pos(:,n1(ii)) - pos(:,n2(ii)))';
    R(ii, (n2(ii)-1)*3 + (1:3)) = (pos(:,n2(ii)) - pos(:,n1(ii)))';
end


end

