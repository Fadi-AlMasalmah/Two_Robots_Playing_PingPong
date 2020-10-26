    function Rx = rotx(alpha_1)

        Rx = [1,         0,          0;
            0, cos(alpha_1), -sin(alpha_1);
            0, sin(alpha_1),  cos(alpha_1)];
    end
    
    
%     Rx = [1,         0,          0, 0;
%             0, cos(alpha_1), -sin(alpha_1), 0;
%             0, sin(alpha_1),  cos(alpha_1), 0;
%             0,         0,          0, 1];