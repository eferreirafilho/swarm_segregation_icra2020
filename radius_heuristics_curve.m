%Input: Radclu
%Output: 
%      For Cluster Segregation -> Updated Shift
function [shift_ca] = radius_heuristics_curve(A_var,wg,N,M,shift_ca)
    
    n_abs=size(M,2);
  
    %for each robot i
    for i=1:N
        %Neighbors of robot i
        vec_A=A_var(i,1:N)';
        %for each neighbor
        for j=1:N
            %If robot i communicates with robot j
            if vec_A(j)==1
            if (wg(i)) > (wg(j))
                %Only if the new value is bigger than the one I already have
                if (shift_ca(j,1)) >= shift_ca(i,1)
                    shift_ca(i,1)=shift_ca(j,1)+1;
                end
            end
            %Robots of the same group
            if (wg(i)) == (wg(j))
                %Only if the new value is bigger than the one I already have
                if (shift_ca(j,1)) >= shift_ca(i,1)
                    shift_ca(i,1)=shift_ca(j,1);
                end
            end
            end
        end
    end
    %Transform Variables
    cont=1;
    for k=1:n_abs       
        shift(1:M(k),k)=shift_ca(cont:(cont+M(k)-1));
        cont=cont+M(k);
    end

