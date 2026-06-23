%% analyse data
Totalcost = []; 
mass = 545.18; 
data_space = logspace(-1,2,50);
k_space = mass*data_space; 

for i = 1:numel(k_space)
    str = append(num2str(k_space(i)),'_k_horse_extended_12ms.mat'); 
    Data = load(str); 
    Totalcost = [Totalcost, Data.Data.Cost.COT]; 

end
% clf
% figure()
hold on
plot(data_space,Totalcost)

title('Rotary gallop at 12ms for horse')
xlabel('Spring stiffness normalised by body mass')
ylabel('Cost of transport')

xscale log

% hold on