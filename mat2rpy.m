function  [rpy] = rot2rpy(rotM)
% Filippo Brizzi
if length(rotM) == 4
    rotM = rotM(1:3,1:3);
end

%p = asin(rotM(1,3));
%r = atan2(-rotM(2,3),rotM(3,3));
%y = atan2(-rotM(1,2),rotM(1,1));
q = rot2quat(rotM);
q = quatnormalize(q);
x = q(1);
y = q(2);
z = q(3);
w = q(4);
rpy(1) = atan2(2 *(w*x + y*z), 1 - 2  *(x^2 + y^2));
rpy(2) = asin(2 * (w*y - z*x));
rpy(3) = atan2(2 *(w*z + x*y), 1 - 2 * (y^2 + z^2));
end

function q = rot2quat(rotM)
Qx = rotM(1, 1);
Qy = rotM(2, 2);
Qz = rotM(3, 3);
t = Qx + Qy + Qz;
if t > 0
	    w = 0.5 * sqrt(1 + t);
	    x = (rotM(3, 2) - rotM(2, 3)) / (4 * w);
	    y = (rotM(1, 3) - rotM(3, 1)) / (4 * w);
	    z = (rotM(2, 1) - rotM(1, 2)) / (4 * w);
elseif (rotM(1, 1) > rotM(2, 2)) & (rotM(1, 1) > rotM(3, 3))
	S = sqrt(1 + rotM(1, 1) - rotM(2, 2) - rotM(3, 3)) * 2;
	w = (rotM(3, 2) - rotM(2, 3)) / S;
		x = 0.25 * S;
		y = (rotM(1, 2) + rotM(2, 1)) / S;
		z = (rotM(1, 3) + rotM(3, 1)) / S;
	elseif rotM(2, 2) > rotM(3, 3)
		S = sqrt(1 + rotM(2, 2) - rotM(1, 1) - rotM(3, 3)) * 2;
		w = (rotM(1, 3) - rotM(3, 1)) / S;
		x = (rotM(1, 2) - rotM(2, 1)) / S;
		y = 0.25 * S;
		z = (rotM(2, 3) + rotM(3, 2)) / S;
	else
		S = sqrt(1 + rotM(3, 3) - rotM(1, 1)  - rotM(2, 2)) * 2;
		w = (rotM(2, 1) - rotM(1, 2)) / S;
		x = (rotM(1, 3) + rotM(3, 1)) / S;
		y = (rotM(2, 3) + rotM(3, 2)) / S;
		z = 0.25 * S;
	end
q = [x, y, z, w];
end

function n = quatnormalize(q)
%QUATNORMALIZE Normalize a quaternion.
%   N = QUATNORMALIZE(Q) computes N, the result of normalizing the
%   quaternion Q.

n = bsxfun(@rdivide, q, sqrt(sum(q.^2, 2)));
end