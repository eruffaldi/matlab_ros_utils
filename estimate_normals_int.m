% if the cloud is integral then ...
function nxyz = estimate_normals_int(xyz,depth,r,mode)

% http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php
%http://islab.ulsan.ac.kr/files/announcement/411/Adaptive%20Neighborhood%20Selection%20for%20Real-Time%20Surface%20Normal.pdf
%
%Adaptive Neighborhood Selection for Real-Time Surface Normal
%Estimation from Organized Point Cloud Data Using Integral Images

%ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
%ne.setMaxDepthChangeFactor(0.02f);
%ne.setNormalSmoothingSize(10.0f);

%
% As we see,
%our approach based on smoothed depth changes (SDC)
%has significantly better abilities to handle noise than our
%approach based on covariance matrices (CM) or the kNNbased
%approach, which both use the covariance matrix to
%estimate surface normals

%COVARIANCE_MATRIX,
%  AVERAGE_3D_GRADIENT,
%AVERAGE_DEPTH_CHANGE

%The COVARIANCE_MATRIX mode creates 9 integral images to compute the normal for a specific point from the covariance matrix of its local neighborhood. The AVERAGE_3D_GRADIENT mode creates 6 integral images to compute smoothed versions of horizontal and vertical 3D gradients and computes the normals using the cross-product between these two gradients. The AVERAGE_DEPTH_CHANGE mode creates only a single integral image and computes the normals from the average depth changes.

IPz = integralImage(depth);
S = @(I,m,n,r) (1/4/r^2)*(I(m+r,n+r)-I(m-r,n+r)-I(m+r,n-r)+I(m-r,n-r));
Px = xyz(:,:,1);
Py = xyz(:,:,2);
nxyz = nan(size(xyz),'double');
switch(mode)
    case 'cov'
    case 'avg3d'        
    case 'avgdepth' 
    for m=1+r:size(Px,1)-r
        for n=1+r:size(Py,1)-r
            vphx = (Px(m+r,n)- Px(m-r,n))/2;
            vphy = (Py(m+r,n)- Py(m-r,n))/2;
            %regionSum = J(eR+1,eC+1) - J(eR+1,sC) - J(sR,eC+1) + J(sR,sC)
            vphz = (S(IPz,m+1,n,r-1)-S(IPz,m-1,n,r-1))/2;
            vpvx = (Px(m,n+r)- Px(m,n-r))/2;
            vpvy = (Py(m,n+r)- Py(m,n-r))/2;
            vpvz = (S(IPz,m,n+1,r-1)-S(IPz,m,n-1,r-1))/2;
            vph = [vphx,vphy,vphz];
            vpv = [vpvx,vpvy,vpvz];
            np = cross(vph,vpv);
            nxyz(m,n,:) = np/norm(np);
        end
    end
end
