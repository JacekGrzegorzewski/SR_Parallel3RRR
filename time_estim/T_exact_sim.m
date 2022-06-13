function [T] = T_exact_sim(steps,acc)

CLK = 84000000;
Alpha = 2*pi/6400;


T =floor(0.676*CLK*sqrt(2*Alpha/acc));

rest =0;
Tot = T;
 for n = 1:1:steps
        tmp = rest;
        rest =  mod (2*T+tmp,4*n+1) ;
        T = T - floor((2*T+tmp)/(4*n+1));
        Tot = Tot+T;
 end

T= Tot/CLK;



end

