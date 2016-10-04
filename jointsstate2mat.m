function m = jointstates2mat(j)


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
m.joints = v;

