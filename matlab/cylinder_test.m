Xc = [0.5,0,5];

r = .18;    %radius of robot
height = .4;%height of robot

dim = [640,480];    %[width,height] of image

K = [575.8157348632812, 0.0, 314.5; 0.0, 575.8157348632812, 235.5; 0.0, 0.0, 1.0];

h_squared = Xc(1)^2 + Xc(3)^2;
h = sqrt(h_squared);

theta_c = atan2(Xc(1),Xc(3));
theta_d = asin(r/h);

Xt_l = [h*sin(theta_c - theta_d), Xc(2), h*cos(theta_c - theta_d)]';
Xt_r = [h*sin(theta_c + theta_d), Xc(2), h*cos(theta_c + theta_d)]';

U_l = Xt_l/Xt_l(3);
U_r = Xt_r/Xt_r(3);

p_l = K * U_l;
p_r = K * U_r;

p_l_int = max(0,floor(p_l));
p_r_int = min(ceil(p_r),dim(1));

p_reproj = p_l_int;


for p_x = p_l_int(1)+1:p_r_int(1)-1
    
    p_reproj(1) = p_x; %coordinates of pixel we need to calculate depth for
    
    L = K\p_reproj;    %reprojecting pixel to ray

    %equations for intersection of ray and circle
    a = L(1)^2 + L(3)^2;
    b = -2*(L(1)*Xc(1) + L(3)*Xc(3));
    c = h_squared - r^2;
    
    if b^2-4*a*c < 0
        disp('Error! Complex solution!\n');
        return;
    end

    %solve for parameter t that yields intersection
    %Note that we only care about the more distant intersection 
    %(the + solution)
    t = (-b + sqrt(b^2-4*a*c))/(2*a);
    
    %get world coordinates of intersection
    xh = L*t;
    
    %get depth for that pixel
    depth = xh(3);
    
    

end