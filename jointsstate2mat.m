function [m,jn] = jointstates2mat(j,separate)

if nargin == 1
    separate = 0;
end

v = zeros(length(j),length(j{1}.position));
t = zeros(length(j),1);
for I=1:length(j)
    p = j{I}.position;
    t(I) = j{I}.header.stamp.time;
    v(I,:) = p;
end

m = dataset();
m.time = t;
m.mtime = unix2matlab(t);

jn = j{1}.name;
jn = cell2struct(num2cell(1:length(jn))',jn);

if separate
    ff = fieldnames(jn);
    for I=1:length(ff)
        m.(ff{I}) = v(:,jn.(ff{I}));
    end
else
    m.joints = v;
end
    
