function M = ros_pq2mat(p,q)
% http://docs.ros.org/jade/api/tf/html/c++/Quaternion_8h_source.html
% quaternion is stored as: xyzw http://docs.ros.org/jade/api/tf/html/c++/QuadWord_8h_source.html
% http://docs.ros.org/jade/api/tf/html/c++/Matrix3x3_8h_source.html
% tfScalar d = length2;
%00149                 tfFullAssert(d != tfScalar(0.0));
%00150                 tfScalar s = tfScalar(2.0) / d;
%00151                 tfScalar xs = x * s;   ys = y * s;   zs = z * s;
%00152                 tfScalar wx = w * xs;  wy = w * ys;  wz = w * zs;
%00153                 tfScalar xx = x * xs;  xy = x * ys;  xz = x * zs;
%00154                 tfScalar yy = y * ys;  yz = y * zs;  zz = z * zs;
%00155                 setValue(tfScalar(1.0) - (yy + zz), xy - wz, xz + wy,
%00156                         xy + wz, tfScalar(1.0) - (xx + zz), yz - wx,
%00157                         xz - wy, yz + wx, tfScalar(1.0) - (xx + yy));

d = norm(q);
s = 2.0/d;
x = q(1);
y = q(2);
z = q(3);
w = q(4);
xs = x * s;   ys = y * s;   zs = z * s;
wx = w * xs;  wy = w * ys;  wz = w * zs;
xx = x * xs;  xy = x * ys;  xz = x * zs;
yy = y * ys;  yz = y * zs;  zz = z * zs;

R = [1.0-(yy+zz),xy-wz,xz+wy; xy+wz,1.0-(xx+zz),yz-wx; xz-wy,yz+wx,1.0-(xx+yy)];
M = eye(4);
M(1:3,1:3) = R;
M(1:3,4) = p;