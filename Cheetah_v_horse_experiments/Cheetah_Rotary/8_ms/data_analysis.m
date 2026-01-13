%% Analyse data
Totalcost = []; 

mass = 45.5; 
data_space = logspace(-1,2,50);
k_space = mass*data_space; 

for i = 1:numel(k_space)
    str = append(num2str(k_space(i)),'_k_cheetah_extended_8ms.mat'); 
    Data = load(str); 
    Totalcost = [Totalcost, Data.Data.Cost.COT]; 

end
%% Plot COT vs stiffness
% clf
figure()
plot(data_space,Totalcost)

title('Rotary gallop at 8 m/s for cheetah')
xlabel('Spring stiffness (k/m)')
ylabel('COT')

xscale log

% hold on
