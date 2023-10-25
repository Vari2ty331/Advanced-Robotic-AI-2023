function [ ovec ] = planning_mem_2_whole_mem( ivec, fixed_index, length_of_fixed_index)
%PLANNING_POS_2_WHOLE_POS 이 함수의 요약 설명 위치
%   자세한 설명 위치

ovec = zeros(length(ivec) + length(fixed_index), 1);
ovec_bool = zeros(size(ovec));

for i=1:length(fixed_index)
    ovec(fixed_index(i)) = length_of_fixed_index(i);
    ovec_bool(fixed_index(i)) = 1;
end

cnt = 1;
for i=1:length(ovec)
    if ~ovec_bool(i)
        ovec(i) = ivec(cnt);
        cnt = cnt + 1;
    end
end


end

