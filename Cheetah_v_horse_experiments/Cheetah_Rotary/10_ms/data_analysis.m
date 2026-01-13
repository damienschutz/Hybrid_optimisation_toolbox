%% analyse data
Totalcost = []; 
mass = 45.5; 
data_space = logspace(-1,2,50);
k_space = mass*data_space; 

for i = 1:numel(k_space)
    str = append(num2str(k_space(i)),'_k_cheetah_extended_10ms.mat'); 
    Data = load(str); 
    Totalcost = [Totalcost, Data.Data.Cost.COT]; 

end
% clf
% figure()
hold on
plot(data_space,Totalcost)

title('Rotary gallop at 10ms for cheetah')
xlabel('Spring stiffness normalised by body mass')
ylabel('Cost of transport')

xscale log

% hold on