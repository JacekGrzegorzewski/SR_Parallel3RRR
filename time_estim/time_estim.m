clear;
clc;
close all;

CLK = 84000000;
Alpha = 2*pi/6400;


NUM=175;
num = 0;
acc_plus= 0000;
steps_plus=000;
step=0.01;

 z_ex=zeros(NUM-num,NUM);
 z_app=zeros(NUM-num,NUM);
X = (num+1):1:(NUM+steps_plus);
Y =  1:1:((NUM+acc_plus));

for i= (num+1):1:(NUM+steps_plus)
    for j = step:step:((NUM+acc_plus)*step)
        z_app(i-num,int16(j/step)) = T_approx_sim(i,j);
        z_ex(i-num,int16(j/step)) =  T_exact_sim(i,j);
    end
end


 z=(z_app-z_ex);

figure('Name','Surf1','NumberTitle','off');
surf(z)
xlabel('acc'), ylabel('steps'), zlabel('T_ex-T_app')



% Z = 1:1:100;

% 
% steps=[1:1:9 , 10:10:50];
% t=1:1:NUM;
% figure
% hold on
% for i = 1:1:length(steps)
%     for n = steps:steps:NUM*steps
%         Z(n) = (T_approx_sim(steps(i),n)-T_exact_sim(steps(i),n));
%     end
%     plot(t,Z,'DisplayName',num2str(steps(i)));
% end
% xlabel("Acc[0.01]"), ylabel("Err")
% legend show
% hold off 



% t = 1:1:(NUM+steps_plus);
% 
% plots = [1, 10:10:100];
% 
% figure
% hold on;
% for i = [1, 10:10:70]
% plot(t,z(:,int16(i)),'DisplayName',num2str(i*step));
% end
% xlabel('steps'), ylabel('err')
% legend show
% 

figure('Name','Fig2','NumberTitle','off');

t = 1:1:(NUM+acc_plus);



ft = fittype("a/sqrt(x)+b");

AB=zeros(2,20);
LIM = 20;
for i = 1:LIM
hold on;
hold on;
plot(t,z(int16(i),:),'DisplayName',num2str(i));
end
xlabel('acc'), ylabel('err')
legend show


 z_estim=zeros(NUM-num,NUM);

for i= (num+1):1:(NUM+steps_plus)
    for j = step:step:((NUM+acc_plus)*step)
        z_estim(i-num,int16(j/step)) = correction(i,j);
    end
end


z_corr=z_estim-z;

figure
title("error-estim")
surf(z_corr);
xlabel('steps'), ylabel('acc'), zlabel('T_ex-T_app')

for i= (num+1):1:(NUM+steps_plus)
    for j = step:step:((NUM+acc_plus)*step)
        z_estim(i-num,int16(j/step)) = correction1(i,j);
    end
end

z_corr=z_estim-z;

figure
title("error-estim1")
surf(z_corr);
xlabel('steps'), ylabel('acc'), zlabel('T_ex-T_app')




function T = T_exact_sim(steps,acc)
CLK = 84000000;
Alpha = 2*pi/6400;


if(acc<0.001)
    acc=0.001;
end

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

function T = T_approx_sim(steps,acc)
if(acc<0.001)
    acc=0.001;
end
Alpha = 2*pi/6400;
T=sqrt(2*Alpha*steps/acc);
end



function err = correction(steps,acc)

err = 0.1*((-0.1956)*steps^(-0.4198)+0.1559*0.9885)/sqrt(acc);

end

function err = correction1(steps,acc)

    err = 0.1*((9.221)*steps^(0.00495)-9.123)/sqrt(acc*6.303);

end

function T = T_approx(steps,acc,decc,speed)

CLK = 84000000;
Alpha = 2*pi/6400;

speed=min(speed,CLK*Alpha/3000);

accel_stop = floor(speed^2/(2*Alpha*acc));
if(accel_stop<1)
    accel_stop=1;
end
peak_velocity = floor(steps*decc/(acc+decc));
if(peak_velocity<1)
    peak_velocity=1;
end

if(accel_stop<=peak_velocity)
    T=speed/acc;
else
    T=sqrt(2*Alpha*peak_velocity/acc);
end



end

function T = T_exact(steps,acc,decc,speed)


CLK = 84000000;
Alpha = 2*pi/6400;

speed=min(speed,CLK*Alpha/3000);

accel_stop = floor(speed^2/(2*Alpha*acc));
if(accel_stop<1)
    accel_stop=1;
end

peak_velocity = floor(steps*decc/(acc+decc));
if(peak_velocity<1)
    peak_velocity=1;
end

if(accel_stop<=peak_velocity)
    dec_start = steps-accel_stop*acc/decc;
else
    dec_start = peak_velocity;
end

T =floor(0.676*CLK*sqrt(2*Alpha/acc));

rest =0;
Tot = T;
 for n = 1:1:min(accel_stop,dec_start)
        tmp = rest;
        rest =  mod (2*T+tmp,4*n+1) ;
        T = T - floor((2*T+tmp)/(4*n+1));
        Tot = Tot+T;
 end

T= Tot/CLK;





end