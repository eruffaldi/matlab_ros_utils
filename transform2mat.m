function r = transform2mat(j,t)

    po = zeros(length(j),3);
    qo = zeros(length(j),4);
    mo = zeros(length(j),16);
    if nargin == 1
      t = zeros(length(j),1);
    else
        t = t(:);
    end
    for I=1:length(j)
        if isstruct(j) == 0
            p = j{I}.transform.translation;
            q = j{I}.transform.rotation;
            t(I) = j{I}.header.stamp.time;
        else
            p = j(I).translation;
            q = j(I).rotation;
        end
        po(I,:) = p;
        qo(I,:) = q;
        x = ros_pq2mat(p,q);
        mo(I,:) = x(:);
    end

    r = dataset();
    r.time = t;
    r.mtime = unix2matlab(t);

    r.mat = mo;
    r.quat = qo;
    r.pos = po;
