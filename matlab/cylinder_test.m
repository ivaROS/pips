Xc = [.6,.3,1.5]; %coordinates of hallucinated robot origin in camera frame (right = x, down = y, out = z)

r = .18;    %radius of robot
height = .4;%height of robot

camera_height = .3;

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
Xt_rt = Xt_rb + [0,-height,0]';


U_lb = Xt_lb/Xt_lb(3);  %ray to bottom left point
U_rb = Xt_rb/Xt_rb(3);  %ray to bottom right point
U_lt = Xt_lt/Xt_lt(3);  %ray to top left point
U_rt = Xt_rt/Xt_rt(3);  %ray to top right point

p_lb = K * U_lb;
p_rb = K * U_rb;
p_lt = K * U_lt;
p_rt = K * U_rt;


p_l_int = max(1,floor(p_lb(1))); %remember to use 0 indexing in C
p_r_int = min(ceil(p_rb(1)),dim(1));

p_b_int = min(ceil(p_rb(2)),dim(2));
p_t_int = max(1,floor(p_lt(2)));

row_range = p_t_int:p_b_int;

simulated_image(row_range,p_l_int) = Xt_lb(3);
simulated_image(row_range,p_r_int) = Xt_rb(3);

for p_x = p_l_int+1:p_r_int-1
    
    %reprojecting x pixel coordinate at center of y coordinates to ray
    L = K\[p_x;K(2,3);1];
    
    %%Equivalent to the below 
    %L = K\p_reproj;    %reprojecting pixel to ray
    %L(2) = 0;
    
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
    X_h = L*t;
    
    %for bottom:
    X_hb = X_h + [0,camera_height,0]';
    
    %for top:
    X_ht = X_h + [0,camera_height-height,0]';
    
    %project back to pixels to get y coordinate
    U_xht = X_ht/X_ht(3);
    p_xht = K*U_xht;
    
    U_xhb = X_hb/X_hb(3);
    p_xhb = K*U_xhb;
    
    %get depth for that pixel
    depth = X_h(3);
    
    row_range = max(1,floor(p_xht(2))):min(dim(2),ceil(p_xhb(2)));
    
    simulated_image(row_range,p_x) = depth;
end

figure(1);
imagesc(simulated_image);