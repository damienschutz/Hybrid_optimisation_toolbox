% Quadruped model script - flexible roll inclusion
function model = Cheetah()

% % clc;
% clear; 

% Generalise the massess and legs
total_length = 0.75; 
total_mass = 45.5; 
% m_torso = 0.65*2/3*total_mass; 
% m_trunk = 0.65*1/3*total_mass; l_torso = 2/3*total_length; l_trunk = 1/3*total_length;
w_torso = 0.717/pi; w_trunk = 0.717/pi; 
m_torso = 19.7167; m_trunk = 9.8583; l_torso = 0.5; l_trunk = 0.25; 

% m_front_leg = (15.2/2)*(1/100)*total_mass; l_humerous = 0.243; m_humerous = 0.164;  
% m_humerous = 2.5687;
%
m_total_forelimb = 2.5846; l_forelimb = 0.243; w_forelimb = 0.025; 
m_foot_forelimb = 1.1204; 

m_total_hindlimb = 3.34915; l_hindlimb = 0.262; w_hindlimb = 0.0227; 
m_hindlimb_foot = 1.50335; 


% w_humerous = 0.025; l_radius = 0.232; m_radius = 0.063; w_radius = 0.0171; 


% m_hind_leg = (19.8/2)*(1/100)*total_mass; l_femur = 0.262; l_tibia = 0.262;
% 
% m_femur = 0.196; w_femur = 0.0227; 

% m_tibia = 0.152; w_tibia = 0.0184; 

% m_extension_front = 0.527; 
% m_extension_hind = 0.538; 
m_extension_front = 0.232; 
m_extension_hind = 0.262;

% Floating Base
jtype{1} = 'Px'; 
Xtree{1} = plux(eye(3), [0; 0; 0]); 
I{1} = mcI(0,[0;0;0],0); 
appearance.body{1} = {};

jtype{2} = 'Py'; 
Xtree{2} = plux(eye(3), [0; 0; 0]); 
I{2} = mcI(0,[0;0;0],0); 
appearance.body{2} = {};

jtype{3} = 'Pz'; 
Xtree{3} = plux(eye(3), [0; 0; 0]); 
I{3} = mcI(0,[0;0;0],0); 
appearance.body{3} = {};

jtype{4} = 'Rx'; 
Xtree{4} = plux(eye(3), [0; 0; 0]); 
I{4} = mcI(0,[0;0;0],0); 
appearance.body{4} = {};

jtype{5} = 'Ry'; 
Xtree{5} = plux(eye(3), [0; 0; 0]); 
I{5} = mcI(0,[0;0;0],0); 
appearance.body{5} = {};

% Torso
I_torso = eye(3); 
I_torso(1,1) = 1/2*m_torso*(w_torso/2)^2; I_torso(2,2) = 1/12*m_torso*(3*(w_torso/2)^2 + l_torso^2);
I_torso(3,3) = 1/12*m_torso*(3*(w_torso/2)^2 + l_torso^2);
jtype{6} = 'Rz';
Xtree{6} = plux(eye(3),[0;0;0]);
I{6} = mcI(m_torso,[0;0;0],I_torso);  
appearance.body{6} = {'colour',[255/255 174/255 28/255],'cyl', [l_torso/2 0 0;-l_torso/2 0 0],w_torso/2}; 

% Trunk 
I_trunk = eye(3); 
I_trunk(1,1) = 1/2*m_trunk*(w_trunk/2)^2; I_trunk(2,2) = 1/12*m_trunk*(3*(w_trunk/2)^2 + l_trunk^2);
I_trunk(3,3) = 1/12*m_trunk*(3*(w_trunk/2)^2 + l_trunk^2);
I{7} = mcI(m_trunk,[-l_trunk/2;0;0],I_trunk);  
jtype{7} = 'Ry';
Xtree{7} = plux(eye(3),[-l_torso/2;0;0]);
appearance.body{7} = {'colour',[255/255 174/255 28/255],'cyl', [0 0 0;-l_trunk 0 0],w_trunk/2}; 

% Forelimbs
% Front Right
% Massless Leg 
jtype{8} = 'Rx'; 
Xtree{8} = plux(eye(3), [l_torso/2; -w_torso/2-w_forelimb/2; 0]); % fix offset here? 
I{8} = mcI(0,[0;0;0],0); 
appearance.body{8} = {};

% Hip joint
I_front_leg = eye(3); 
I_front_leg(1,1) = 1/12*m_total_forelimb*(3*(w_forelimb/2)^2 + l_forelimb^2); 
I_front_leg(2,2) = 1/12*m_total_forelimb*(3*(w_forelimb/2)^2 + l_forelimb^2); 
I_front_leg(3,3) = 1/2*m_total_forelimb*((w_forelimb/2)^2); 
jtype{9} = 'Ry'; 
Xtree{9} = eye(6); 
I{9} = mcI(m_total_forelimb,[0;0;-l_forelimb/2],I_front_leg);  
appearance.body{9} = {'colour',[255/255 174/255 28/255],'cyl', [0 0 0; 0 0 -l_forelimb], w_forelimb/2};

% Knee joint
I_front_foot = 2/3*m_foot_forelimb*(w_forelimb/2)^2*eye(3); 
jtype{10} = 'Pz'; 
Xtree{10} = plux(ry(pi), [0;0;-l_forelimb]); 
I{10} = mcI(m_foot_forelimb, [0;0;0], I_front_foot);
appearance.body{10} = {{'colour',[0.7 0.7 0.7],'cyl',[0 0 0; 0 0 -m_extension_front], w_forelimb/2*0.7},{'colour',[0 0 0],'sphere', [0 0 0], w_forelimb/2}};

% Front Left
% Massless Leg 
jtype{11} = 'Rx'; 
Xtree{11} = plux(eye(3), [l_torso/2; w_torso/2+w_forelimb/2; 0]); % fix offset here? 
I{11} = mcI(0,[0;0;0],0); 
appearance.body{11} = {};

% Hip joint
jtype{12} = 'Ry'; 
Xtree{12} = eye(6); 
I{12} = mcI(m_total_forelimb,[0;0;-l_forelimb/2],I_front_leg);  
appearance.body{12} = {'colour',[255/255 174/255 28/255],'cyl', [0 0 0; 0 0 -l_forelimb], w_forelimb/2};

% Knee joint
jtype{13} = 'Pz'; 
Xtree{13} = plux(ry(pi), [0;0;-l_forelimb]); 
I{13} = mcI(m_foot_forelimb, [0;0;0], I_front_foot);
appearance.body{13} = {{'colour',[0.7 0.7 0.7],'cyl',[0 0 0; 0 0 -m_extension_front], w_forelimb/2*0.7},{'colour',[0 0 0],'sphere', [0 0 0], w_forelimb/2}};

% Hind limbs
% Hind right
% Massless Leg 
jtype{14} = 'Rx'; 
Xtree{14} = plux(eye(3), [-l_trunk; -w_trunk/2-w_hindlimb/2; 0]); % fix offset here? 
I{14} = mcI(0,[0;0;0],0); 
appearance.body{14} = {};

% Hip joint
I_hind_leg = eye(3); 

I_hind_leg(1,1) = 1/12*(m_total_hindlimb)*(3*(w_hindlimb/2)^2+l_hindlimb^2); 
I_hind_leg(2,2) = 1/12*(m_total_hindlimb)*(3*(w_hindlimb/2)^2+l_hindlimb^2); 
I_hind_leg(3,3) = 1/2*(m_total_hindlimb)*((w_hindlimb/2)^2); 

jtype{15} = 'Ry'; 
Xtree{15} = eye(6); 
I{15} = mcI((m_total_hindlimb),[0;0;-l_hindlimb/2],I_hind_leg);  
appearance.body{15} = {'colour',[255/255 174/255 28/255],'cyl', [0 0 0; 0 0 -l_hindlimb] w_hindlimb/2};

% Knee joint
I_hind_foot = 2/3*m_hindlimb_foot*(w_hindlimb/2)^2*eye(3); 
jtype{16} = 'Pz'; 
Xtree{16} = plux(ry(pi), [0 0 -l_hindlimb]); 
I{16} = mcI((m_hindlimb_foot), [0;0;0], I_hind_foot);
appearance.body{16} = {{'colour',[0.7 0.7 0.7],'cyl',[0 0 0; 0 0 -m_extension_hind], w_hindlimb/2*0.7},{'colour',[0 0 0],'sphere', [0 0 0] w_hindlimb/2}};

% Hind left
% Massless Leg 
jtype{17} = 'Rx'; 
Xtree{17} = plux(eye(3), [-l_trunk; w_trunk/2+w_hindlimb/2; 0]); % fix offset here? 
I{17} = mcI(0,[0;0;0],0); 
appearance.body{17} = {};

% Hip joint
jtype{18} = 'Ry'; 
Xtree{18} = eye(6); 
I{18} = mcI(m_total_hindlimb,[0;0;-l_hindlimb/2],I_hind_leg);  
appearance.body{18} = {'colour',[255/255 174/255 28/255],'cyl', [0 0 0; 0 0 -l_hindlimb] w_hindlimb/2};

% Knee joint
jtype{19} = 'Pz'; 
Xtree{19} = plux(ry(pi), [0 0 -l_hindlimb]); 
I{19} = mcI((m_hindlimb_foot), [0;0;0], I_hind_foot);
appearance.body{19} = {{'colour',[0.7 0.7 0.7],'cyl',[0 0 0; 0 0 -m_extension_hind], w_hindlimb/2*0.7},{'colour',[0 0 0],'sphere', [0 0 0] w_hindlimb/2}};

% Ground contacts
gc.body = [10,13,16,19]; 
gc.point = [[0;0;0],[0;0;0],[0;0;0],[0;0;0]]; 
gc.name = {'FR','FL','HR','HL'};

appearance.base = { 'tiles', [-50 50; -50 50; 0 0], 0.5 };

NB = 19; 
parent = [0 1 2 3 4 5 6 6 8 9 6 11 12 7 14 15 7 17 18];

model.NB = NB; 
model.parent = parent; 
model.Xtree = Xtree; 

model.I = I; 
model.jtype = jtype;
model.appearance = appearance; 
model.gc = gc; 
% showmotion(model)

end 
