%Input: Robots Positions (rrx,rry,rrz(if 3D))
%Rb-> Robot Size
%rd->Region where robots are colliding, tipically 2*Rb
%rc->Region where collision avoidance works (smaller than communication radius)

%Output: Controllers with collision avoidance

function [min_dist,varargout] = collision_avoidance_potfn(Rb,rd,rc,N,rrx_ca,rry_ca,rrz_ca)

if nargin==6
    dim=2;
else
    dim=3;
end

%Preallocating Variables
Vx_ca(1:N,1)=0;
Vy_ca(1:N,1)=0;
if dim==3
    Vz_ca(1:N,1)=0;
end
V_ca(1:N,1)=0;
%Vx_ma(1:n_robots*n_abs,1)=0;
%Vy_ma(1:n_robots*n_abs,1)=0;
%V_ma=0;
%V_fa=0;
%Vfx(1:n_robots*n_abs,1)=0;
%Vfy(1:n_robots*n_abs,1)=0;   
%if dim==3
    %Fazer depois
%end
    

if rd<=rc
    disp('rd must be between rc and c')
    pause
end   
%ra=5*Rb;
%gamma_ma=0.7;
%tal=1;%Quality of the communication
%aij(1:n_robots*n_abs,1:n_robots*n_abs)=0;
%df=4.2;
%c1=2;
%c2=5;
%kf=50;

min_dist=10000;  
Qcol=600000;%Parameter

for i=1:N
    for j=1:N
         if i~=j
            norm_r=sqrt((rrx_ca(i) - rrx_ca(j))^2 + (rry_ca(i) - rry_ca(j))^2);
            if dim==3
                norm_r=sqrt((rrx_ca(i) - rrx_ca(j))^2 + (rry_ca(i) - rry_ca(j))^2 + (rrz_ca(i) - rrz_ca(j))^2);
            end
            if norm_r<min_dist
                min_dist=norm_r;
            end
            %Collision Avoidance
            if norm_r>rc && norm_r<rd
                %Based on Li & Wang 2013
                %V_ca(i,1)=((4*((rd)^2-(rc)^2)*((rd)^2-norm_r^2))/((norm_r^2-(rc)^2))^3);
                %Vx_ca(i,1)=Vx_ca(i,1)+V_ca(i,1)*(rrx_ca(i)-rrx_ca(j));
                %Vy_ca(i,1)=Vy_ca(i,1)+V_ca(i,1)*(rry_ca(i)-rry_ca(j));
                %if dim==3
                %   Vz_ca(i,1)=Vz_ca(i,1)+V_ca(i,1)*(rrz_ca(i)-rrz_ca(j));
                %end
                %Based on Mondal et. al. 2018
                 V_ca(i,1)=(norm_r)/(((norm_r^2-rc)^2) + (1/Qcol));
                 Vx_ca(i,1)=Vx_ca(i,1)+V_ca(i,1)*(rrx_ca(i)-rrx_ca(j));
                 Vy_ca(i,1)=Vy_ca(i,1)+V_ca(i,1)*(rry_ca(i)-rry_ca(j));
                 if dim==3
                    Vz_ca(i,1)=Vz_ca(i,1)+V_ca(i,1)*(rrz_ca(i)-rrz_ca(j));
                 end 
            end
            if norm_r<2*Rb
                disp('Collision')
                %pause
            end
         end
    end
end
    
varargout{1}=Vx_ca;
varargout{2}=Vy_ca;
if dim==3
    varargout{3}=Vz_ca;
end
    

       
            
    %% Collision Avoidance proof tentative
    
    %Laplacian of collision avoidance region
%     if dim==2
%         %Adjacency matrix
%         A_til=adj_mat_calculate(rrx_ca,rry_ca,rd);
%         %Laplacian Matrix
%         L_til=diag(sum(A_til,2))-A_til;
%     else
%         %Adjacency matrix
%         A_til=adj_mat_calculate3D(rrx_ca,rry_ca,rrz_ca,rd);
%         %Laplacian Matrix
%         L_til=diag(sum(A_til,2))-A_til;
%     end
    
    %Proof
%     small_gamma_with_ca=[zeros(n_robots*n_abs),eye(n_robots*n_abs);-L_fixed-L_fixed*V_ca,-k_gamma*L_fixed];%
%     eig_small_gamma_with_ca(:,t)=sort(eig(small_gamma_with_ca));
%     cont_eig=0;
%     if real(eig_small_gamma_with_ca(1,t))<0.0001 && real(eig_small_gamma_with_ca(2,t))<0.0001
%         cont_eig=2;
%         for i=3:2*size(L_fixed,1)
%             if real(eig_small_gamma_with_ca(i,t))<0
%                 cont_eig=cont_eig+1;
%             end    
%         end
%     end
%     if cont_eig~=2*size(L_fixed,1)
%         %disp('not OK')
%     end
 
            
%             %Flexible connectivity maintenence
%             if norm_r<(tal*c)
%                 aij(i,j)=1;
%             end
%             if norm_r>=(tal*c) && norm_r<c
%                 aij(i,j)=0.5*(1+cos(pi*(((norm_r/c)-tal)/(1-tal))));
%             end
% %                     if norm_r<=df
% %                         Vf=-((2*c1)/(lambda2^2))*(abs((1/norm_r) - (1/df))^(2*c1-1));
% %                         Vfx(i,1)=Vfx(i,1) + Vf*((rrx_ca(j)-rrx_ca(i))/norm_r);
% %                         Vfy(i,1)=Vfy(i,1) + Vf*((rrx_ca(j)-rrx_ca(i))/norm_r);
% %                     end
% %                     if norm_r>df && norm_r<=c
% %                         Vf=((kf*pi)/((2*(lambda2^(c2)))*(c-df)))*sin(pi*(norm_r-df)/(c-df));
% %                         Vfx(i,1)=Vfx(i,1) + Vf*((rrx_ca(j)-rrx_ca(i))/norm_r);
% %                         Vfy(i,1)=Vfy(i,1) + Vf*((rrx_ca(j)-rrx_ca(i))/norm_r);
% %                     end
%             if flag_ca==1    
%                 %Collision Avoidance
%                 if norm_r>rc && norm_r<rd
%                     V_ca(i,1)=((4*((rd)^2-(rc)^2)*((rd)^2-norm_r^2))/((norm_r^2-(rc)^2))^3);
%                     Vx_ca(i,1)=Vx_ca(i,1)+V_ca(i,1)*(rrx_ca(i)-rrx_ca(j));
%                     Vy_ca(i,1)=Vy_ca(i,1)+V_ca(i,1)*(rry_ca(i)-rry_ca(j));
%                     if dim==3
%                         Vz_ca(i,1)=Vz_ca(i,1)+V_ca(i,1)*(rrz_ca(i)-rrz_ca(j));
%                     end                       
%                 end
%             end
%             if flag_ma==1    
%                 %Maintain Connectivity
%                 if norm_r>rc && norm_r<gamma_ma*ra
%                     %V_ma=-((4*((ra)^2-(rd)^2)*((ra)^2-norm_r^2))/((norm_r^2-(rd)^2))^3);
%                     %Vx_ma(i,1)=Vx_ma(i,1)-V_ca*(rrx_ca(j)-rrx_ca(i));
%                     %Vy_ma(i,1)=Vy_ma(i,1)-V_ca*(rry_ca(j)-rry_ca(i));
%                     V_ma=((gamma_ma^2)*(c^2) - norm_r^2)^2;
%                     Vx_ma(i,1)=Vx_ma(i,1)+(rrx_ca(j)-rrx_ca(i))/V_ma;
%                     Vy_ma(i,1)=Vy_ma(i,1)+(rry_ca(j)-rry_ca(i))/V_ma;
%                 end
%             end
%             
%         end
%     end     
% end