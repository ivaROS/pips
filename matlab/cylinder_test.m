Xc = [-0.3,0,2]; %coordinates of robot origin in camera frame (right = x, down = y, out = z)

r = .18;    %radius of robot
height = .4;%height of robot

dim = [640,480];    %[width,height] of image

simulated_image = ones(dim(2),dim(1)) * Xc(3);

%Default kinect calibration values used by ROS
K = [575.8157348632812, 0.0, 314.5; 0.0, 575.8157348632812, 235.5; 0.0, 0.0, 1.0];

h_squared = Xc(1)^2 + Xc(3)^2;
h = sqrt(h_squared);

theta_c = atan2(Xc(1),Xc(3));
theta_d = asin(r/h);

Xt_lb = [h*sin(theta_c - theta_d), Xc(2), h*cos(theta_c - theta_d)]';
Xt_rb = [h*sin(theta_c + theta_d), Xc(2), h*cos(theta_c + theta_d)]';

Xt_lt = Xt_lb + [0,-height,0]';

U_lb = Xt_lb/Xt_lb(3);
U_rb = Xt_rb/Xt_rb(3);
U_lt = Xt_lt/Xt_lt(3);

p_lb = K * U_lb;
p_rb = K * U_rb;
p_lt = K * U_lt;

p_l_int = max(1,floor(p_lb(1))); %remember to use 0 indexing in C
p_r_int = min(ceil(p_rb(1)),dim(1));

p_b_int = min(ceil(p_rb(2)),dim(2));
p_t_int = max(1,floor(p_lt(2)));

row_range = p_t_int:p_b_int;

p_reproj = round(p_lb);

simulated_image(row_range,p_l_int) = Xt_lb(3);
simulated_image(row_range,p_r_int) = Xt_rb(3);

for p_x = p_l_int+1:p_r_int-1
    
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
    
    simulated_image(row_range,p_reproj(1)) = depth;
end

figure(1);
imagesc(simulated_image);