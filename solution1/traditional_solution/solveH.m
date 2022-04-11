function [H,res] = solveH(m,M)
for i = 1:size(M,2)
    L(2*i-1,:) = [M(:,i)' zeros(1,3) -m(1,i)*M(:,i)'];
    L(2*i  ,:) = [zeros(1,3) M(:,i)' -m(2,i)*M(:,i)'];
end
[~,~,V] = svd(L);
H0 = V(:,end);
H0 = H0/H0(9);
H0 = H0';
options = optimset('Algorithm','levenberg-marquardt');
[x,res] = lsqnonlin(@toMinimize, H0, [], [], options, m, M);
H = reshape(x,3,3)';
end
