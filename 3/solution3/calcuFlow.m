function [u,v] = calcuFlow(lastFrame,frame,surfPoints)
Ix_m = conv2(rgb2gray(lastFrame),[-1 1; -1 1], 'valid'); % partial on x
Iy_m = conv2(rgb2gray(lastFrame), [-1 -1; 1 1], 'valid'); % partial on y
It_m = conv2(rgb2gray(lastFrame), ones(2), 'valid') + conv2(rgb2gray(frame), -ones(2), 'valid');
u = zeros(length(surfPoints),1);
v = zeros(length(surfPoints),1);
w=10;
for k = 1:length(surfPoints.Location(:,2))
    i = round(surfPoints.Location(k,2));
    j = round(surfPoints.Location(k,1));
    Ix = Ix_m(i-w:i+w, j-w:j+w);
    Iy = Iy_m(i-w:i+w, j-w:j+w);
    It = It_m(i-w:i+w, j-w:j+w);
    
    Ix = Ix(:);
    Iy = Iy(:);
    b = -It(:); % get b here
    
    A = [Ix Iy]; % get A here
    nu = pinv(A)*b;
    
    u(k)=nu(1);
    v(k)=nu(2);
end
end

