function [ q__,q_,q ] = BangBangThresh( t , Ti,Tf ,q_max, qi , qf )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% didn't use Tj from the input but instead I calculated it to reach q_max
T = Tf - Ti;
D = qf - qi;
t = t-Ti; %%%%%%%%%
Tm = (T)/2;
q_max = q_max* sign(D);

if(abs(D)<= T/2 * abs(q_max))      %didn't reach the threashold (q_max : max Velocity - Degrees)
    qm_ = 2*D/T;
    m = 4*D/T^2;
    if(t<Tm)
     q_ = m*t;
     q__ = m;
     q = qi + m*t^2/2;
    else
      q_ = qm_ - m*(t-Tm);
      q__ = -m;
      q = qi + m*Tm^2/2 + qm_*(t-Tm)-m*(t-Tm)^2/2;
    end
else                                      % the velocity reaches the Threshold
    Tj = T - D/q_max;
    % if(Tj > Ttotal/2)
    %     Tj = Ttotal/2;
    % end
    m = q_max/Tj;
    
    if( t< Tj)
        q_ = m*(t);
        q__ = m;
        q = qi + m*t^2/2;
    elseif(t< T - Tj)
            q_ = q_max;
            q__ = 0;
            q = qi + m*Tj^2/2 + (t-Tj)*q_max;
        else
            q_ = q_max - m*((t) - (T - Tj));
            q__ = -m;
            q = qi + m*Tj^2/2 + q_max*(T-2*Tj) + q_max*(t-(T - Tj)) - m*(t-(T - Tj))^2/2;
    end
end
% q__ = 0;
end


