%% analyse data
Totalcost = []; 
mass = 545.18; 
data_space = logspace(-1,2,50);
k_space = mass*data_space; 
x_f = []; 

for i = 1:numel(k_space)
    str = append(num2str(k_space(i)),'_k_horse_grounded_10ms.mat'); 
    Data = load(str); 
    Totalcost = [Totalcost, Data.Data.Cost.COT]; 
    x_f = [x_f, Data.Data.PhaseData(8).position(end,1)]; 

end
% Totalcost = Totalcost.*x_f;
% clf
% figure()
hold on
plot(data_space,Totalcost)

title('Grounded transverse gallop at 10ms for horse')
xlabel('Spring stiffness normalised by body mass')
ylabel('Cost of transport')

xscale log

% hold on