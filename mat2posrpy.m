function r = mat2posrpy(M)

r = [];
r.pos = M(1:3,4)';
r.rpy = mat2rpy(M);