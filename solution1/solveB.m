function [vCol1,vCol2] = solveB(H)
    v12=[H(1,1)*H(1,2) ,H(1,1)*H(2,2)+H(2,1)*H(1,2) ,H(2,1)*H(2,2) ,H(3,1)*H(1,2)+H(1,1)*H(3,2), H(3,1)*H(2,2)+H(2,1)*H(3,2), H(3,1)*H(3,2)];
    v11=[H(1,1)^2 2*H(2,1)*H(1,1) H(2,1)^2 2*H(3,1)*H(1,1) 2*H(3,1)*H(2,1) H(3,1)^2];
    v22=[H(1,2)^2 2*H(2,2)*H(1,2) H(2,2)^2 2*H(3,2)*H(1,2) 2*H(3,2)*H(2,2) H(3,2)^2];
    vCol1=reshape(v12,1,6);
    vCol2=v11-v22;
end
