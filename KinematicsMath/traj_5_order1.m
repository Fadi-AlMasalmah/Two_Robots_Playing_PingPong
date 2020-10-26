function [qdd,qd,q] = traj_5_order1(t,traj_Cs,n)  
            tt = [ones(n,1) t t.^2 t.^3 t.^4 t.^5]';
            q = traj_Cs*tt;
            
            ttd = [zeros(n,1) ones(n,1) 2*t 3*t.^2 4*t.^3 5*t.^4]';
            qd = traj_Cs*ttd;
            
            ttdd = [zeros(n,1) zeros(n,1) 2*ones(n,1) 6*t 12*t.^2 20*t.^3]';
            qdd = traj_Cs*ttdd;
end


