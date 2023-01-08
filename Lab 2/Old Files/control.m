path_m_ref = test(true);

%%

load('DataFiles/path_m_ref.mat')
path_m_ref = path_m_ref(1:end,:);

n = size(path_m_ref,1);

K_v = 0.5;
K_s = 100;
K_i = 1;
h = .001;
L = 2.2;

x_traj = zeros(size(path_m_ref,1),1);
y_traj = zeros(size(path_m_ref,1),1);
theta_traj = zeros(size(path_m_ref,1),1);
v_traj = zeros(size(path_m_ref,1),1);
w_traj = zeros(size(path_m_ref,1),1);
phi_traj = zeros(size(path_m_ref,1),1);

x_ref = path_m_ref(2,1);
x = path_m_ref(1,1);
% x = -5;
y_ref = path_m_ref(2,2);
y = path_m_ref(1,2);
% y = 1;
theta_ref = atan2(y_ref-y,x_ref-x);
theta = path_m_ref(1,3);

phi = theta_ref - theta;
    
tol = 0.1;
it = 1;
% while(abs(x-path_m_ref(end,1)) > tol || abs(y-path_m_ref(end,2)) > tol)
while(it < n)
    
    w_e = [x_ref-x, y_ref-y, theta_ref-theta];
    b_e = [cos(theta), sin(theta), 0; -sin(theta), cos(theta), 0; 0, 0, 1]*w_e';

    v = K_v*b_e(1);
    w_s = K_s*b_e(3)+K_i*b_e(2);
    
    if w_s > 1.5
        w_s = 1.5;
    elseif w_s < -1.5
        w_s = -1.5;
    end
        
%     phi = theta_ref-theta;
%     if abs(phi) > pi/4
%         if phi > 0
%             phi = phi/4;
%         else
%             phi = -phi/4;
%         end
%     end

    phi_traj(it) = phi;
    
    A = [cos(theta), 0; sin(theta) 0; tan(phi)/L 0; 0 1];

    state_deriv = A*[v;w_s];
    
    dt = h;

    x = x + state_deriv(1)*dt;
    y = y + state_deriv(2)*dt;
    theta = theta + state_deriv(3)*dt;

    phi = phi + w_s*dt;
    if phi > pi/4
        phi = pi/4;
    elseif phi < -pi/4
        phi = -pi/4;
    end
       
    x_traj(it) = x;
    y_traj(it) = y;
    theta_traj(it) = theta;
    v_traj(it) = v;
    w_traj(it) = w_s;
    
    dist = sqrt((x-x_ref)^2+(y-y_ref)^2);
    
    if dist > 3
        continue;
    end
    
    it = it + 1;
    
    % theta_ref = path_m_ref(it,3);
    theta_ref = atan2(y_ref-y,x_ref-x);
    x_ref = path_m_ref(it,1);
    y_ref = path_m_ref(it,2);
    
end

figure(1)
scatter(path_m_ref(:,1), path_m_ref(:,2), 5, 'r')
hold on;
scatter(x_traj, y_traj, 5, 'b')
xlim([100,160])
ylim([50,200])

figure(2)
plot(v_traj, 'r')
max(v_traj)
min(v_traj)
% 
% figure(3)
% plot(theta_traj, 'r')
% 
% figure(4)
% plot(w_traj, 'r')
% 
% figure(5)
% plot(phi_traj, 'r')
