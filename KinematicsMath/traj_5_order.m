function [qdd,qd,q] = traj_5_order(t,traj_Cs)  
            tt = [1 t t^2 t^3 t^4 t^5]';
            q = traj_Cs*tt;
            
            ttd = [0 1 2*t 3*t^2 4*t^3 5*t^4]';
            qd = traj_Cs*ttd;
            
            ttdd = [0 0 2 6*t 12*t^2 20*t^3]';
            qdd = traj_Cs*ttdd;
end


