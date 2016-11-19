function nxyz = estimate_normals(xyz,target,vp,k)

Mdl = KDTreeSearcher(xyz);
nxyz = zeros(length(target),3);

for I=1:length(target)
    p = target(I,:);
    w = knnsearch(Mdl,p,'K',k);
    P = xyz(w,:);
    Pm = (P-repmat(mean(P),size(P,1),1));
    
    C = (Pm'*Pm)/length(w);
    [V,E] = eig(C); 
    %[~,J] = min(diag(E))
    J =1;
    n = V(:,J);
    n = n/norm(n);
    if n'*(vp-p)' < 0
        n = -n;
    end
    nxyz(I,:) = n;
end
