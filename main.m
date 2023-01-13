
close all
clear variables

M=[2,1,3,2,2,1];%Number of robots per group
n_abs=size(M,2);
N=sum(M);
MM=max(M);

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);
Rb=0.055;

%Fixed seed
rng(1)

%Collision Avoidance [1] Yes [0] No
flag_ca=1;
%Segregation Controller Parameters
c_sensing=0.30;
%Treating collision
rd=0.15;

d(1:N,1)=0;
d_gain=0.0003;
k_gamma=3;%Velocity dissipation gain
kp=1;%Consensus gain
%shift_ca_var=[1;1;1;2;2;2;3;3;3;4;4;5;5];
%shift_ca_var=[0;0;1;1;2;2];
shift_ca_var(1:N,1)=0;
%Spiral parameter
a=140;
b=1.4;

%% Experiment constants
%Fixed connected topology
L = random_connectedGL(N,1);
A = eye(N).*L - L;

%% Grab tools we need to convert from single-integrator to unicycle dynamics
% Gain for the diffeomorphism transformation between single-integrator and
% unicycle dynamics
[~, uni_to_si_states] = create_si_to_uni_mapping();
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion();

uni_barrier_cert_boundary = create_uni_barrier_certificate_with_boundary();

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 5000;

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dxi = zeros(2, N);

% Initialize dxu
dxu = zeros(2, N);

%Initialize other variables
vvx_ca(1:N,1)=0;
vvy_ca(1:N,1)=0;
UUx_ca(1:N,1)=0;
UUy_ca(1:N,1)=0;
rrx_ca_hist(1:N,1:iterations)=0;
rry_ca_hist(1:N,1:iterations)=0;
vvx_ca_hist(1:N,1:iterations)=0;
vvy_ca_hist(1:N,1:iterations)=0;
UUx_ca_hist(1:N,1:iterations)=0;
UUy_ca_hist(1:N,1:iterations)=0;

info_consensus(1:N,1)=rand(N,1);
shift_ca(1:N,1)=0;

%% Which Group Function
aux_ca_wg=(1:1:N);

cont=1;
for k=1:n_abs
    %wg(n_robots*k-(n_robots-1):n_robots*k)=aux_ca_wg(:,k);
    wg(cont:(cont+M(k)-1))=aux_ca_wg(:,k);
    cont=cont+M(k);
end

% Get initial location data for while loop condition.
x=r.get_poses();
%Set Color
rand_color(1:10,1:3)=[1 0 0;0 1 0;0 0 1;0 1 1;0 0 0;0.3 0.5 0.7;0.3 0.7 0;1 1 0;0.9 0.9 0.9;0.3 0.3 0.3];
%Set marker size
marker_size_robot = determine_robot_marker_size(r);

list_markers=['p','d','s','h','o','x'];
marker_size=7;
%Plot Spiral
max_tt=max(d.*shift_ca_var);%Max size of spiral
rx=mean(x(1,:));
ry=mean(x(2,:));
cont=1;
for tt=0:0.01:max_tt 
   x_spi(cont)=sqrt(tt).*cos(sqrt(a.*tt))*b-rx;
   y_spi(cont)=sqrt(tt).*sin(sqrt(a.*tt))-ry;
   %x_spi(cont)=sqrt(tt)-rx;
   %y_spi(cont)=-ry;
   cont=cont+1;
end
h1=plot(x_spi,y_spi,'--k','LineWidth',4,'color','k');

for i=1:N
    % Plot colored circles showing robot location.
    g(i) = plot(x(1,i),x(2,i),list_markers(wg(i)),'MarkerSize', marker_size_robot,'LineWidth',7,'Color',rand_color(wg(i),:));
    %Plot Sensing Radius
    theta = 0 : 0.02 : 2*pi;
    xA = c_sensing * cos(theta) + x(1,i);
    yA = c_sensing * sin(theta) + x(2,i);
    h2(i)=plot(xA, yA,'color',rand_color(wg(i),:),'LineWidth',1);
end


% for i=1:N
%     h3(i)=line([x(1,i) 0],[x(2,i) 0],'color',rand_color(wg(i),:),'LineWidth',1);
% end

%% Converge to a region Initialization
w_search(1:N,1)=0;
w(1:N,1:2)=0;


i_step=0.033;
r.step();
tic

%Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    % Convert to SI states
    xi = uni_to_si_states(x);
    
    %Get velocities (New way)
    vvx_ca(1:N,1)=(dxu(1,:).*cos(x(3,:)));
    vvy_ca(1:N,1)=(dxu(1,:).*sin(x(3,:)));
    
    %Get Positions
    rrx_ca(1:N,1)=x(1,:);
    rry_ca(1:N,1)=x(2,:);
    
    %Adjacency Matrix
    A_ca=adj_mat_calculate(rrx_ca,rry_ca,c_sensing);
    
    
    %% Heuristics
    [shift_ca] = radius_heuristics_curve(A_ca,wg,N,M,shift_ca);
    
    shift_ca_var=shift_ca.*d;%Parameter obtained via consensus
    
    %% Chosen Curve
    sx=sqrt(shift_ca_var).*cos(sqrt(a.*shift_ca_var))*b;
    sy=sqrt(shift_ca_var).*sin(sqrt(a.*shift_ca_var));
    
    % Update Plotting Information and Locations
    for i = 1:N
        g(i).XData = x(1,i);
        g(i).YData = x(2,i);
        
        %Plot Sensing Radius
        theta = 0 : 0.02 : 2*pi;
        xA = c_sensing * cos(theta) + rrx_ca(i);
        yA = c_sensing * sin(theta) + rry_ca(i);
        % Plot colored circles showing robot sensing radius
        h2(i).XData = xA;
        h2(i).YData = yA;
    end
    
    % Update spiral to plot 
    max_tt=max(shift_ca_var);%Max size of spiral
    rx=mean(rrx_ca);
    ry=mean(rry_ca);
    cont=1;
    for tt=0:0.01:max_tt
        x_spi(cont)=sqrt(tt).*cos(sqrt(a.*tt))*b+rx;
        y_spi(cont)=sqrt(tt).*sin(sqrt(a.*tt))+ry;
        cont=cont+1;
    end
    h1.XData = x_spi;
    h1.YData = y_spi;
    
    %% Converge to a region instead of a point
    for i=1:N
        for j=1:N
            if j>i
                %If robots are seeing each other
                if A_ca(i,j)==1
                    %If robots are from the same group
                    if wg(i)==wg(j)
                        norm_w=sqrt((w(i,1)-w(j,1))^2+(w(i,2)-w(j,2))^2);   
                        %If robots have conflicting points
                        if norm_w<=rd
                            %Robot i pick a new random value with increased search area
                            w_search(i,1)=w_search(i,1)+0.001;
                            w(i,:)=-w_search(i,1) + (w_search(i,1)+w_search(i,1)).*rand(2,1);
                            w_search(j,1)=w_search(j,1)+0.001;
                            w(j,:)=-w_search(j,1) + (w_search(j,1)+w_search(j,1)).*rand(2,1);
                        end

                    end
                end
            end
        end
    end
    
    %z=q+s+w
    zx_ca=rrx_ca-sx+w(:,1);
    zy_ca=rry_ca-sy+w(:,2);
    
     %for i = 1:N
        %plot virtual points
        %h3(i).XData = [rrx_ca(i) zx_ca(i)];
        %h3(i).YData = [rry_ca(i) zy_ca(i)];
     %end
    
    %% Increasing d
    %While any enemy robot in sight, increase d
    for i=1:N
        %All robots robot i sees
        vec_ind=A_ca(:,i);      
        
        %Number of robots of other groups processed so far -> The same plus M(i)
        vec_ind(numel((wg(wg<wg(i))))+1:numel((wg(wg<wg(i))))+1+M(wg(i)))=0;    
        
        if sum(vec_ind)>0 %Robot i is seeing at least one "enemy"
            d(i,1)=d(i,1)+d_gain;%Increase variable 
        end
    end  
    
    %% Consensus
    % Get the topological neighbors of agent i based on the graph
    %Laplacian L
    %neighbors = topological_neighbors(L, );
    for i=1:N
        for j=1:N 
            %Relative Damping 2nd Order Consensus
            UUx_ca(i)=UUx_ca(i)-kp*A(i,j)*((zx_ca(i)-zx_ca(j)));
            UUy_ca(i)=UUy_ca(i)-kp*A(i,j)*((zy_ca(i)-zy_ca(j)));
            %Information consensus
            %A1 Consensus Protocol (Average Consensus)
            d(i,1)=d(i,1)+0.1*A(i,j)*(d(j,1)-d(i,1));    
        end
    end
    %Damping Term
    UUx_ca=UUx_ca-k_gamma*vvx_ca;
    UUy_ca=UUy_ca-k_gamma*vvy_ca;
    
    
    %% Collision Avoidance
    if flag_ca==1
        %Parameters
        rc=0.11;%Collision
        [min_dist_aux,Vx_ca,Vy_ca] = collision_avoidance_potfn(Rb,rd,rc,N,rrx_ca,rry_ca);
        UUx_ca=UUx_ca+Vx_ca;
        UUy_ca=UUy_ca+Vy_ca;
    end
    
    %% Save hist
    rrx_ca_hist(:,t)=rrx_ca;
    rry_ca_hist(:,t)=rry_ca;
    vvx_ca_hist(:,t)=vvx_ca;
    vvy_ca_hist(:,t)=vvy_ca;
    UUx_ca_hist(:,t)=UUx_ca;
    UUy_ca_hist(:,t)=UUy_ca;
    
    %% Transform the double-integrator controller to single-integrator dynamics
    UUx_ca=vvx_ca+i_step*UUx_ca;
    UUy_ca=vvy_ca+i_step*UUy_ca;
       
    %% Applying controllers to the robots
    dxi(1,:)=UUx_ca;
    dxi(2,:)=UUy_ca;
    
    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    %% Map SI to Uni dynamics and utilize barrier certificates
    
    % Transform the single-integrator to unicycle dynamics using the the
    % transformation we created earlier
    dxu = si_to_uni_dyn(dxi, x);
    
    dxu = uni_barrier_cert_boundary(dxu, x);
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!    
    r.step();
    
    % Resize Marker Sizes (In case user changes simulated figure window
    % size, this is unnecessary in submission as the figure window 
    % does not change size).
    marker_size_robot = num2cell(ones(1,N)*determine_robot_marker_size(r));
end

%Save data
save('alldata.mat')

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();

toc

%% Helper Functions

% Marker Size Helper Function to scale size of markers for robots with figure window
% Input: robotarium class instance
function marker_size = determine_robot_marker_size(robotarium_instance)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Points');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
robot_ratio = (robotarium_instance.robot_diameter + 0.03)/...
    (robotarium_instance.boundaries(2) - robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * robot_ratio;

end
