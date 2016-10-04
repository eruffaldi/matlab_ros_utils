function M = celltf2mat(c)

M = zeros(length(c),16);
for I=1:length(c)
    m = ros_pq2mat(c(I).translation,c(I).rotation);
    M(I,:) = m(:);
end
