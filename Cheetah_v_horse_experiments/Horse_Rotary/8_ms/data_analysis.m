%% analyse data
Totalcost = []; 
mass = 545.18; 
data_space = logspace(-1,2,50);
k_space = mass*data_space; 
xf = []; 

for i = 1:numel(k_space)
    str = append(num2str(k_space(i)),'_k_horse_extended_8ms.mat'); 
    Data = load(str); 
    Totalcost = [Totalcost, Data.Data.Cost.COT]; 
    xf = [xf,Data.Data.PhaseData(8).position(end,1)]; 

end
% clf
% figure()

% Totalcost = Totalcost.*xf*5348.22; 
hold on
plot(data_space,Totalcost)

title('Rotary gallop at 8ms for horse')
xlabel('Spring stiffness normalised by body mass')
ylabel('Cost of transport')

xscale log

% hold on