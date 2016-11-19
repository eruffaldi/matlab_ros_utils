function [xyz,rgb] = make_point_cloud(r,d,ci,dense,limitz)

if nargin < 4
    dense = 0;
end

if ischar(dense)
    dense = strcmp(dense,'dense');
end
if nargin < 5
    limitz = [0,Inf];
end

assert(size(r,1) == size(d,1) & size(r,2) == size(d,2),'same sizes');

fx = ci.K(1);
cx = ci.K(3);
fy = ci.K(5);
cy = ci.K(6);
D = cast(d,'double')/1000.0;
if dense
    warning('missing limit')
    warning('missing function to flatten wh into one axis later for visualization')
    xyz = zeros(size(d,1),size(d,2),3,'double');
    rgb = cast(r,'double')/255.0;
    for I=1:size(d,1)
        for J=1:size(d,2)
            Z = D(I,J);
            if Z < limitz(1) | Z > limitz(2)
                xyz(I,J,:) = NaN;
            else
                X = (I-cx)*Z/fx;
                Y = (J-cy)*Z/fy;
                xyz(I,J,:) = [X,Y,Z];
            end
        end
    end

else
    k = sum(sum(D > limitz(1) & D < limitz(2)));
    xyz = zeros(k,3); % double
    rgb = zeros(k,3); % dounle
    % estimate count
    k = 1;
    for I=1:size(d,1)
        for J=1:size(d,2)
            Z = D(I,J);
            if Z > limitz(1) & Z < limitz(2)
                X = (I-cx)*Z/fx;
                Y = (J-cy)*Z/fy;
                xyz(k,:) = [X,Y,Z];
                rgb(k,:) = r(I,J,:);
                k = k + 1;
            end
        end
    end
    rgb = rgb/255.0;
end