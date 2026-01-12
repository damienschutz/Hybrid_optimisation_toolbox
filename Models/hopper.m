%% Model design - recursive newton-euler set up - kinematic tree

function model = hopper()
    m = 1; g = 1; l = 1;                 % main parameters

    mf = 0.05*m; ml = 0.1*m; mt = 0.85*m;               % masses
    If = 0.002*m*l^2; Il = 0.002*m*l^2; It = 0.4*m*l^2; % inertias
    kalpha = 5*m*g*l; kl = 10*m*g/l;                    % spring coefficients
    bl = 0.2*sqrt(2)*m*sqrt(g/l); balpha = 0.2*mf*g*l;  % damping
    rf = 0.05*l; dl = 0.25*l; df = 0.25*l;              % geometry

    % Ground appearance
    appearance.base = { 'tiles', [-10 10; 0 0; -10 10], 0.1 };
    
    % Floating base
    I{1} = mcI(0,[0 0], 0); 
    jtype{1} = 'px'; 
    Xtree{1} = plnr(0, [0 0]); 

    I{2} = mcI(0,[0 0], 0); 
    jtype{2} = 'py'; 
    Xtree{2} = plnr(0, [0 0]);     

    % Base
    I{3} = mcI(mt, [0 0], It); 
    jtype{3} = 'r'; 
    Xtree{3} = plnr(0, [0 0]); 
    appearance.body{3} = {'colour',[0.12941 0.471 0.267] , 'box', [-l/2 -dl/2 -dl/2; l/2 dl dl/2]}; 

    % Leg
    I{4} = mcI(ml,[0 -dl], Il); 
    jtype{4} = 'r'; 
    Xtree{4} = plnr(0, [0 0]); 
    appearance.body{4} = { 'box', [-rf 0 rf; rf -l+df -rf]}; 

    % Foot
    I{5} = mcI(mf, [0 df], If);
    jtype{5} = 'py'; 
    Xtree{5} = plnr(0, [0 -l]);
    appearance.body{5} = {'colour',[0.12941 0.471 0.267], {'colour',[0.12941 0.471 0.267], 'sphere', [0 0 0], rf},{'cyl', [0 0 0;0 2*df 0], rf/2}}; 

    % Ground contact
    gc.body = 5; 
    gc.point = [0; -rf]; 
    gc.name = "Foot"; 

    model.NB = 5; 
    model.parent = [0 1 2 3 4]; 
    model.Xtree = Xtree; 
    model.I = I; 
    model.jtype = jtype; 
    model.appearance = appearance; 
    model.gravity = [0;-1]; 
    model.gc = gc; 

    % showmotion(model)

end