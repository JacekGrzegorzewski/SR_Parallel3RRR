function [outputArg1] = T_approx_sim(steps,acc)
%T_APPROX_SIM Summary of this function goes here


Alpha = 2*pi/6400;
outputArg1=sqrt(2*Alpha*steps/acc);

end

