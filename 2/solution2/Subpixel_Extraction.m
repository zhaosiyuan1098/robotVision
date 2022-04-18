function sub_edge=Subpixel_Extraction(edge,originImg)

if size(originImg,3)==3
    originImg=rgb2gray(originImg);
end

originImg=double(originImg);


mask_r=2;
mask=fspecial('gaussian',2*mask_r+1);
gaussianImg=imfilter(originImg,mask,'replicate');


w=fspecial('sobel');
gradient_h_im=imfilter(gaussianImg,w,'replicate');
w=w';
gradient_w_im=imfilter(gaussianImg,w,'replicate');
gradient_amp_im=sqrt(gradient_h_im.^2+gradient_w_im.^2);


R=[-2 -1 0 1 2 ]; 
Len=length(edge);
Neigh_H_Matrix=zeros(Len,5); 
Neigh_W_Matrix=zeros(Len,5);
Gradient_Thea=zeros(Len,1);


for i=1:Len
    h=edge(i,1);            
    w=edge(i,2);
    dh=gradient_h_im(h,w);
    dw=gradient_w_im(h,w);
    thea=atan2(dh,dw);
    Neigh_H_Matrix(i,:)=R*sin(thea)+h;
    Neigh_W_Matrix(i,:)=R*cos(thea)+w;
    Gradient_Thea(i)=thea;
    
end

Neigh_Gray_Diff=interp2(gradient_amp_im,Neigh_W_Matrix,Neigh_H_Matrix);

Ln_Neigh_Gray_Diff=log(Neigh_Gray_Diff);

V=[4 -2 1;
   1 -1 1;
   0  0 1;
   1  1 1;
   4  2 1];
E=(V'*V)\V';
sub_pixel_edge=zeros(Len,2);
for i=1:Len
    U=Ln_Neigh_Gray_Diff(i,:)';
    A=E(1,:)*U;
    B=E(2,:)*U;
    x=-B/(2*A);
    sub_pixel_edge(i,1)=edge(i,1)+x*sin(Gradient_Thea(i));
    sub_pixel_edge(i,2)=edge(i,2)+x*cos(Gradient_Thea(i));
end

sub_edge=sub_pixel_edge;
    
    
