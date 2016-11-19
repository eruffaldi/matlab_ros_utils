%%
xyz = zeros(10,10,3);
[x,y] = meshgrid(1:10,1:10);
xyz(:,:,1) = x;
xyz(:,:,2) = y;
nxyz = estimate_normals_int(xyz,d,2,'avgdepth');
nxyz2 = dense2flat(nxyz);
xyz2= dense2flat(xyz);
scatter3(xyz2(:,1),xyz2(:,2),xyz2(:,3));


%%
[x,y] = meshgrid(0:0.1:2*pi,-pi:0.1:pi);
xyz = zeros(size(x,1),size(x,2),3);
r = 2.0;
xyz(:,:,1) = r*cos(x).*sin(y);
xyz(:,:,2) = r*sin(x).*sin(y);
xyz(:,:,3) = r*cos(y);
nxyz = estimate_normals_int(xyz,4,'avgdepth');
nxyz2 = dense2flat(nxyz);
xyz2= dense2flat(xyz);
scatter3(xyz2(:,1),xyz2(:,2),xyz2(:,3));
hold on
quiver3(xyz2(:,1),xyz2(:,2),xyz2(:,3),nxyz2(:,1),nxyz2(:,2),nxyz2(:,3));
hold off
axis equal
