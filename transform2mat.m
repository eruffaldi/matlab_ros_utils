function r = transform2mat(j)

    po = zeros(length(j),3);
    qo = zeros(length(j),4);
    mo = zeros(length(j),16);
    t = zeros(length(j),1);
    for I=1:length(j)
        p = j{I}.transform.translation;
        q = j{I}.transform.rotation;
        t(I) = j{I}.header.stamp.time;
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
