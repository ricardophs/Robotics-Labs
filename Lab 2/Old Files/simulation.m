%% Autonomous Cars
close all
clear
clc

%% Vehicle parameters
L = 2.2;
L_r = 0.566;
L_f = 0.566;
d = 0.64;
r = 0.256;
length_car = 3.332;
width = 1.508;
mass = 810;

%% Load Reference Examples
load('path_5.mat');
%path = path(1:50:2000,:);

%% Simulation parameters
% Simulation variables
t_step = 0.01;

% Initial parameters
last_state.x = path(1,1);
last_state.y = path(1,2);
last_state.theta = path(1,3);
last_state.phi = 0;
last_state.V = 0;
last_state.Ws = 0;

% Controller Parameters
K_v = 0.03; % x -> v (prof: 0.03)
K_i = 1; % y -> omega_s (prof: 1)
K_s = 1000; % theta -> omega_s (prof: 100)

% Limitations
v_max = 10; % m/s
w_max = pi/4;

path_x = path(:,1);
path_y = path(:,2);

%% Simulation
counter = 1;
log_state(counter) = last_state;

% Simulation Output
for i=1:length(path)

 x=path_x(i);
 y=path_y(i);
 
 post_e=[last_state.x;last_state.y]-[x;y];
 n_p = norm(post_e);
 n_pe = n_p;
 
 while n_pe>1
  counter = counter + 1;
  
  %Orientação 'ideal'
  beta=atan2(-post_e(2),-post_e(1));

  %Erro de orientação
  alpha=beta-(last_state.theta+last_state.phi);
  if alpha > pi
   alpha = alpha - 2*pi;
  elseif alpha < -pi
   alpha = alpha + 2*pi;
  end
  K_v=10*exp(-abs(90*alpha));
  
  next_state.V=v_max*tanh(K_v*n_p);

  next_state.Ws=w_max*tanh(((1+(K_i*(beta/alpha)))*(tanh(K_v*n_p)/n_p)*sin(alpha)+K_s*tanh(alpha)));
  
  next_state.phi = last_state.phi + t_step .* (next_state.Ws + last_state.Ws);
  next_state.theta = last_state.theta + t_step .* ( (next_state.V.*tan(next_state.phi)/L) + (last_state.V .* tan(last_state.phi)/L) );
  next_state.x = last_state.x + t_step .* ( ( next_state.V.*cos(next_state.theta) ) + ( last_state.V .* cos(last_state.theta) ) );
  next_state.y = last_state.y + t_step .* ( ( next_state.V.*sin(next_state.theta) ) + ( last_state.V .* sin(last_state.theta) ) );
  
  post_e=[next_state.x;next_state.y]-[x;y];
  n_pe = norm(post_e);

  log_state(counter) = next_state;
  last_state = next_state;
 
 end
 
end

%% Plots
figure(1);
hold on
plot(path_x,path_y,'b:');
plot([log_state.x], [log_state.y],'r-');

mean_V = mean([log_state.V]);
max_V = max([log_state.V]);
min_V = min([log_state.V]);
fprintf('Mean V = %.3f; Max V = %.3f; Min V = %.3f\n', mean_V, max_V, min_V);