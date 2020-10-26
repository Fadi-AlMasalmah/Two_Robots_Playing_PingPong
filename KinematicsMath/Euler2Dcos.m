function [R] = Euler2Dcos(Euler)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
Alpha = Euler(1);
Beta = Euler(2);
Gamma  = Euler(3);
sa = sin(Alpha);
ca = cos(Alpha);
sb = sin(Beta);
cb = cos(Beta);
sg = sin(Gamma);
cg = cos(Gamma);

R = [ [ca*cb , ca*sb*sg - sa*cg , ca*sb*cg + sa*sg];
      [sa*cb , sa*sb*sg + ca*cg , sa*sb*cg - ca*sg];
      [-sb   ,        cb*sg     ,    cb*cg        ]];
end

