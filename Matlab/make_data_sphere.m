function make_data_sphere()
% Absolute measurements

focal_length = 526;
img_height = 344;
img_width = 472;
r = 8;
rv = r+1;

psi = pi;

angle = 40;
theta = angle*pi/180;

[X, Y, Z] = sphere(70);
X = X*r;
Y = Y*r;
Z = Z*r;

dtheta = 0.01*pi/180;
dpsi = 1*pi/180;

pose_gt = [];
measurement_gt = [];

while(theta < (180-angle)*pi/180)
    x = rv*sin(theta)*cos(psi);
    y = rv*sin(theta)*sin(psi);
    z = rv*cos(theta);
    v_phi = 0;
    v_theta = 0;
    v_psi = psi+pi;
    cur_pose = [x;y;z;v_phi;v_theta;v_psi];
    cur_T = state2T(cur_pose);
    cur_R = cur_T(1:3,1:3);
    
    theta2 = acos(rv/r*cos(theta));
    
    norm_dir = [sin(theta2)*cos(psi);...
                sin(theta2)*sin(psi);...
                cos(theta2)];
    point = norm_dir * r;
    
    meas_norm = cur_R'*norm_dir;
    meas_pt = pt_transform(invT(cur_T), point);
    meas_d = -meas_norm'*meas_pt;
    cur_meas = [meas_norm ; meas_d];
%     
%     figure(1);
%     surf(X,Y,Z);
%     hold on;    
%     scatter3(point(1),point(2),point(3),'filled');
%     DrawCam(focal_length, img_height, img_width, cur_pose,0.5, 'r'); 
%     draw_plane(cur_pose, focal_length, img_height, img_width, cur_meas);
%     hold off;
%     set(gca,'YDir','reverse');
%     set(gca,'ZDir','reverse');
%     axis equal;
% 
%     figure(2);
%     scatter3(0,0,0);
%     hold on;
%     DrawCam(focal_length, img_height, img_width, zeros(6,1),0.5, 'r'); 
%     draw_plane(zeros(6,1), focal_length, img_height, img_width, cur_meas);
%     hold off;
%     set(gca,'YDir','reverse');
%     set(gca,'ZDir','reverse');
%     axis equal;
%     drawnow;

    pose_gt = [pose_gt cur_pose];
    measurement_gt = [measurement_gt cur_meas];

    theta = theta+dtheta;
    psi = psi+dpsi;

end

delta_pose = zeros(6, size(pose_gt,2)-1);
delta_pose_noise = zeros(6, size(pose_gt,2)-1);


position_noise = 0.0005;
rotation_noise = 0.0002*pi/180;
meas_normal_noise = 0.002;
meas_dist_noise = 0.002;

measurement_n = zeros(6, size(measurement_gt,2));
measurement_n = measurement_gt + [randn(3,size(measurement_gt,2))*sqrt(meas_normal_noise);...
                               randn(1,size(measurement_gt,2))*sqrt(meas_dist_noise)];

measurement_n(1:3,:) = measurement_n(1:3,:)./(sqrt(sum(measurement_n(1:3,:).^2)));
                           
pose_n = zeros(6,size(pose_gt,2));
pose_n(:,1) = pose_gt(:,1);

absolute_measure = zeros(4, size(pose_gt,1));

gravity_meas_noise = 0.002;
depth_meas_noise = 0.0002;

for i = 1:size(pose_gt,2)
    T = state2T(pose_gt(:,i));
    R = T(1:3,1:3);
    gg = [0;0;1];
    gi = R'*gg;
    gi = gi+randn(3,1)*sqrt(gravity_meas_noise);
    gi = gi/norm(gi);
    absolute_measure(1:3,i) = gi;
end

absolute_measure(4,:) = pose_gt(3,:) + randn(1,size(pose_gt,2))*sqrt(depth_meas_noise);

for i = 1:size(pose_gt,2)-1
    T1 = state2T(pose_gt(:,i));
    T2 = state2T(pose_gt(:,i+1));
    T12 = invT(T1)*T2;
    
    d_state = T2state(T12);
    delta_pose(:,i) = d_state;
    
    d_state_n = [randn(3,1)*sqrt(position_noise);...
                 randn(3,1)*sqrt(rotation_noise)];
             
    delta_pose_noise(:,i) = delta_pose(:,i) + d_state_n;
    
                                      
    T1n = state2T(pose_n(:,i));
    T12n = state2T(delta_pose_noise(:,i));
    T2n = T1n*T12n;
    pose_n(:,i+1) = T2state(T2n);
end

figure(3);
% surf(X,Y,Z);
hold on;
plot3(pose_gt(1,:), pose_gt(2,:), pose_gt(3,:));
plot3(pose_n(1,:), pose_n(2,:), pose_n(3,:));
hold off;
set(gca,'YDir','reverse');
set(gca,'ZDir','reverse');
axis equal;


for i=1:size(pose_n,2)
%     figure(1);
%     surf(X,Y,Z);
%     hold on;    
%     plot3(pose(1,1:i), pose(2,1:i), pose(3,1:i),'r');
%     plot3(pose_n(1,1:i), pose_n(2,1:i), pose_n(3,1:i),'b');
% 
%     DrawCam(focal_length, img_height, img_width, pose(:,i) ,0.5, 'r'); 
%     DrawCam(focal_length, img_height, img_width, pose_n(:,i) ,0.5, 'b');
%     
%     draw_plane(pose(:,i), focal_length, img_height, img_width, measurement(:,i),'r');
%     draw_plane(pose_n(:,i), focal_length, img_height, img_width, measurement_n(:,i),'b');
%     
% 
%     hold off;
%     set(gca,'YDir','reverse');
%     set(gca,'ZDir','reverse');
%     axis equal;    
%     drawnow;
%     
%     
%     figure(2);
%     draw_plane(zeros(6,1), focal_length, img_height, img_width, measurement(:,i),'r');
%     hold on;
%     draw_plane(zeros(6,1), focal_length, img_height, img_width, measurement_n(:,i),'b');
%     DrawCam(focal_length, img_height, img_width, zeros(6,1) ,0.5, 'r');
%     hold off;
%     set(gca,'YDir','reverse');
%     set(gca,'ZDir','reverse');
%     axis equal;    
%     drawnow;
end

%% Write file
write_file('odometry_measurement', delta_pose_noise);
write_file('plane_measurement', measurement_n);
write_file('initial_pose',pose_gt(:,1));
write_file('absolute_measurement',absolute_measure);

write_file('gt_pose', pose_gt);
write_file('gt_measurement', measurement_gt);

end

function write_file(file_name, data)
fileID = fopen(strcat(file_name, '.txt'), 'w');
for i=1:size(data,2)
    for j=1:size(data,1)
        fprintf(fileID, '%8.8f', data(j,i));
        if j<size(data,1)
            fprintf(fileID, '\t');
        end
    end
    if i<size(data,2)
        fprintf(fileID,'\n');
    end
end
fclose(fileID);
end

function pt2 = pt_transform(T, pt)
    temp = [pt; ones(1, size(pt,2))];
    pt2 = T*temp;
    pt2 = pt2(1:3,:);
end

function T = state2T(State)
    x = State(1);
    y = State(2);
    z = State(3);
    roll = State(4);
    pitch = State(5);
    yaw = State(6);
    
    R = rpy2R(roll, pitch, yaw);
    T = [R [x,y,z]';...
         0, 0, 0, 1];
end

function iT = invT(T)
R = T(1:3,1:3);

iT = [R', -R'*T(1:3,4);
      0, 0, 0, 1];
end

function state = T2state(T)
    R = T(1:3,1:3);
    theta = -asin(R(3,1));
    phi = atan2(R(3,2)/cos(theta), R(3,3)/cos(theta));
    psi = atan2(R(2,1)/cos(theta), R(1,1)/cos(theta));
    state = [T(1:3,4) ; phi; theta; psi];
end

function R = rpy2R(r, p, y)

    R_y = [cos(y),  -sin(y), 0;...
           sin(y),   cos(y), 0;...
           0,        0,      1];
       
    R_p = [cos(p),   0,      sin(p);...
           0,        1,      0;...
           -sin(p)   0,      cos(p)];
    
    R_r = [1,        0,      0;...
           0,        cos(r), -sin(r);...
           0,        sin(r), cos(r)];
       
    R = R_y*R_p*R_r;
end


function DrawCam(focalL,ImgH,ImgW,state,L,Color)
Origin=[0;0.00;0];

Depth = L;
Height = L*ImgH/(2*focalL);
Width = L*ImgW/(2*focalL);

P1=[Depth;-Width;-Height];
P2=[Depth;-Width;Height];
P3=[Depth;Width;-Height];
P4=[Depth;Width;Height];

Rroll = [1 0 0;0 cos(state(4)),sin(state(4));0,-sin(state(4)),cos(state(4))];
Rpitch = [cos(state(5)),0,-sin(state(5));0,1,0;sin(state(5)),0,cos(state(5))];
Ryaw = [cos(state(6)),sin(state(6)),0;-sin(state(6)),cos(state(6)),0;0,0,1];
R=Rroll*Rpitch*Ryaw;
P1=P1'*R;
P2=P2'*R;
P3=P3'*R;
P4=P4'*R;

plot3([Origin(1)+state(1);Origin(1)+state(1)+P1(1)],[Origin(2)+state(2);Origin(2)+state(2)+P1(2)],[Origin(3)+state(3);Origin(3)+state(3)+P1(3)],Color,'LineWidth',1.5);
plot3([Origin(1)+state(1);Origin(1)+state(1)+P2(1)],[Origin(2)+state(2);Origin(2)+state(2)+P2(2)],[Origin(3)+state(3);Origin(3)+state(3)+P2(3)],Color,'LineWidth',1.5);
plot3([Origin(1)+state(1);Origin(1)+state(1)+P3(1)],[Origin(2)+state(2);Origin(2)+state(2)+P3(2)],[Origin(3)+state(3);Origin(3)+state(3)+P3(3)],Color,'LineWidth',1.5);
plot3([Origin(1)+state(1);Origin(1)+state(1)+P4(1)],[Origin(2)+state(2);Origin(2)+state(2)+P4(2)],[Origin(3)+state(3);Origin(3)+state(3)+P4(3)],Color,'LineWidth',1.5);
plot3([Origin(1)+state(1)+P1(1);Origin(1)+state(1)+P2(1);Origin(1)+state(1)+P4(1);Origin(1)+state(1)+P3(1);Origin(1)+state(1)+P1(1)],...
    [Origin(2)+state(2)+P1(2);Origin(2)+state(2)+P2(2);Origin(2)+state(2)+P4(2);Origin(2)+state(2)+P3(2);Origin(2)+state(2)+P1(2)],...
    [Origin(3)+state(3)+P1(3);Origin(3)+state(3)+P2(3);Origin(3)+state(3)+P4(3);Origin(3)+state(3)+P3(3);Origin(3)+state(3)+P1(3)],Color,'LineWidth',1.5);
end

function draw_plane(state, focal_length, img_height, img_width, plane_model,color)
T = state2T(state);

V1 = [focal_length; img_width/2; -img_height/2];
V2 = [focal_length; img_width/2; img_height/2];
V3 = [focal_length; -img_width/2; img_height/2];
V4 = [focal_length; -img_width/2; -img_height/2];

t1 = -plane_model(4)/(V1'*plane_model(1:3,1));
t2 = -plane_model(4)/(V2'*plane_model(1:3,1));
t3 = -plane_model(4)/(V3'*plane_model(1:3,1));
t4 = -plane_model(4)/(V4'*plane_model(1:3,1));

P1 = V1*t1;
P2 = V2*t2;
P3 = V3*t3;
P4 = V4*t4;

P1 = pt_transform(T, P1);
P2 = pt_transform(T, P2);
P3 = pt_transform(T, P3);
P4 = pt_transform(T, P4);

Ps = [P1 P2 P3 P4 P1];

plot3(Ps(1,:), Ps(2,:), Ps(3,:),color,'linewidth',2);

end

function draw_circle(surfel_p, surfel_n, surfel_r, surfel_c)
% surfel_p : Surfel center position
% surfel_n : Surfel normal direction
% surfel_r : Surfel radius
% surfel_c : Surfel color (if needed)

t = 0:0.1:2*pi;
z = zeros(size(t,2), 1);
x = surfel_r*cos(t');
y = surfel_r*sin(t');

a = [0;0;1];
b = surfel_n;
v = vx(a)*b;
s = sqrt(v'*v);
c = a'*b;
R2 = eye(3) + vx(v) + vx(v)*vx(v)*(1-c)/(s^2);

[R, ~] = calculate_quaternion([0;0;1], surfel_n);
T = [R surfel_p;0 0 0 1];
X = [x';y';z'; ones(1, size(x,1))];
X2 = T*X;
X2 = X2(1:3,:);
X2 = [X2 X2(:,1)];

fill3(X2(1,:), X2(2,:), X2(3,:), surfel_c);
plot3(X2(1,:), X2(2,:), X2(3,:));
end

function M = vx(vec)
    vec = vec/sqrt((vec'*vec));
    v1 = vec(1);
    v2 = vec(2);
    v3 = vec(3);
    M = [0 -v3 v2;...
         v3 0 -v1;...
         -v2 v1 0];
end

function [R, q] = calculate_quaternion(v1, v2)
    v3 = cross(v1,v2);
    if(norm(v3)==0)
        R = eye(3);
        q = 0;
    else
        v3 = v3/norm(v3);
        theta = acos((v1'*v2)/(norm(v1)*norm(v2)));
        etha = cos(theta/2);
        epsilon = v3*sin(theta/2);
        q = [etha; epsilon];
        R = quat2rotm(q');
    end
end
