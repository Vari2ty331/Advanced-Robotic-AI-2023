function StartTree = makeReference(StartTree)

% N = length(StartTree.Node);
% R = zeros(N, length(StartTree.Node(1).Position));
% for i = 1:N
%     R(i,:) = StartTree.Node(i).Position';
% end

N = length(StartTree.Node);
StartTree.reference(N,:) = StartTree.Node(N).Position';
% R = StartTree.reference;