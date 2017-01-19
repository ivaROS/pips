Xc = [.5,.3,2]'; %coordinates of hallucinated robot origin in camera frame (right = x, down = y, out = z)

robot_radius = .178;    %radius of robot
robot_height = .48;%height of robot
floor_tolerance = .05;

dim = [640,480];    %[width,height] of image

simulated_image = ones(dim(2),dim(1)) * Xc(3);

%Default kinect calibration values used by ROS
K = [575.8157348632812, 0.0, 314.5; 0.0, 575.8157348632812, 235.5; 0.0, 0.0, 1.0];

h_squared = Xc(1)^2 + Xc(3)^2;
h = sqrt(h_squared);

left = 0;
right = dim(1);

if(h > robot_radius)
    
    tangentDist = sqrt(h_squared - robot_radius^2);

    theta_c = atan2(Xc(1),Xc(3));
    theta_d = asin(robot_radius/h);

    Xt_lb = [tangentDist*sin(theta_c - theta_d), Xc(2)-floor_tolerance, tangentDist*cos(theta_c - theta_d)]';
    Xt_rb = [tangentDist*sin(theta_c + theta_d), Xc(2)-floor_tolerance, tangentDist*cos(theta_c + theta_d)]';

    
    %algabraic solution
    rootpart = sqrt(Xc(3)^2*robot_radius^2*(Xc(1)^2+Xc(3)^2-robot_radius^2));
    Tx1 = (Xc(1)^3 +Xc(1)*Xc(3)^2-Xc(1)*robot_radius^2 + rootpart)/(Xc(1)^2+Xc(3)^2);
    Tx2 = (Xc(1)^3 +Xc(1)*Xc(3)^2-Xc(1)*robot_radius^2 - rootpart)/(Xc(1)^2+Xc(3)^2);

    rootpart2 = Xc(1)*sqrt(Xc(3)^2*robot_radius^2*(Xc(1)^2+Xc(3)^2-robot_radius^2));
    
    Ty1 = (Xc(1)^2*Xc(3)^2+Xc(3)^4-Xc(3)^2*robot_radius^2 - rootpart2)/(Xc(3)*(Xc(1)^2+Xc(3)^2));
    Ty2 = (Xc(1)^2*Xc(3)^2+Xc(3)^4-Xc(3)^2*robot_radius^2 + rootpart2)/(Xc(3)*(Xc(1)^2+Xc(3)^2));
    
    t1=[Tx1,Ty1]';
    t2 =[Tx2,Ty2]';
    
    %This confirms that the analytic solution is identical to the
    %trigonometric
    Xt_lb';
    Xt_rb';

    t1';
    t2';
    
    Xt_lt = Xt_lb + [0,-robot_height+floor_tolerance,0]';
    Xt_rt = Xt_rb + [0,-robot_height+floor_tolerance,0]';


    U_lb = Xt_lb/Xt_lb(3);  %ray to bottom left point
    U_rb = Xt_rb/Xt_rb(3);  %ray to bottom right point
    U_lt = Xt_lt/Xt_lt(3);  %ray to top left point
    U_rt = Xt_rt/Xt_rt(3);  %ray to top right point

    p_lb = K * U_lb;
    p_rb = K * U_rb;
    p_lt = K * U_lt;
    p_rt = K * U_rt;

    p_lb_x = min(max(1,floor(p_lb(1))),dim(1)); %remember to use 0 indexing in C
    p_lb_y = max(min(dim(2),ceil(p_lb(2))),1);
    p_lb_ind = [p_lb_x; p_lb_y]

    p_lt_ind = max([1;1],min(floor(p_lt(1:2)),dim'))

    p_rb_x = max(1,min(ceil(p_rb(1)),dim(1)));
    p_rb_y = max(min(dim(2),ceil(p_rb(2))),1);
    p_rb_ind = [p_rb_x; p_rb_y]

    p_rt_ind = max([1;1],min(ceil(p_rt(1:2)),dim'))

    left = p_lb_ind(1)+1;
    right = p_rb_ind(1)-1;
    
    top = min(p_lt_ind(2),p_rt_ind(2))+1;
    bottom = min(p_lb_ind(2),p_rb_ind(2))-1;

    simulated_image(p_lt_ind(2):p_lb_ind(2),p_lb_ind(1)) = Xt_lb(3);
    simulated_image(p_rt_ind(2):p_rb_ind(2),p_rb_ind(1)) = Xt_rb(3);

end

%first do top
py = top;
keepGoing = true;

while keepGoing
    
    p_ray = [0;py];
    
    %reprojecting x pixel coordinate at center of y coordinates to ray
    v_ray = K\[p_ray(1);p_ray(2);1];
    v_row = [1;0;0];
    
    %equations for intersection of plane and circle
    
    y = Xc(2)-robot_height;

    
    
    tz = y*L(3)/L(2);
    
        
    if (L(2)^2*Xc(3)^2-L(2)^2*robot_radius-2*L(2)*L(3)*Xc(3)*y+L(3)^2*y^2) < 0
        disp('Complex solution! No intersection, so beginning rectangle\n');
        keepGoing=false;
        rectangleTop = py;
    else

    
    rootPart = sqrt(-L(2)^2*(L(2)^2*Xc(3)^2-L(2)^2*robot_radius-2*L(2)*L(3)*Xc(3)*y+L(3)^2*y^2));
    tx1 = (L(2)^2*Xc(1) - rootPart)/(L(2)^2);
    tx2 = (L(2)^2*Xc(1) + rootPart)/(L(2)^2);
    
    px1 = K*[tx1;y;tz];
    px2 = K*[tx2;y;tz];

    validRange1 = floor(left):px1;
    validRange2 = ceil(px2):right;
    
    simulated_image(py,validRange1) = 10;
        simulated_image(py,validRange2) = 10;

    
    py = py+1;
    end
end

figure(1);
imagesc(simulated_image);
colorbar()

figure(2);

X0 = [0;0;0];
pointList = [X0, Xt_lb, Xc, Xt_rb, X0];

th = 0:pi/50:2*pi;
xunit = robot_radius * cos(th) + Xc(1);
yunit = robot_radius * sin(th) + Xc(3);
h = plot(xunit, yunit, 'r');
hold on

plot(pointList(1,:),pointList(3,:), 'b');
hold off

axis equal
