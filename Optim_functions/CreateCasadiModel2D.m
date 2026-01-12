%% Function to define the CasADi symbolic functions for the optimisation

% Input is a model defined in Featherstones technique https://royfeatherstone.org/spatial/v2/sysmodel.html
% Output is structure that contains all the necessary symbolic functions to
% interface the dynamics with CasADi
% Constrained dynamics must be included seperately using
% ConstrainedDynamics function

% Created by Damien Schutz
% UCT
% 2025/04/23

function obj = CreateCasadiModel2D(obj, model)
    import casadi.*

    NB = model.NB; % degrees of freedom
    if NB > 2
        NA = NB - 2; % number of actuated degrees of freedom
        B = [SX.zeros(2,NA); SX.eye(NA)]; 
    else
        NA = NB; 
        B = SX.eye(NA); 
    end
    NC = numel(model.gc.body); % number of contact points

    % Create the decision variables
    q = SX.sym('q',NB);
    dq = SX.sym('dq',NB);
    ddq = SX.sym('ddq',NB); 
    x = [q;dq]; 
    u = SX.sym('u',NA);

    % Symbolics 
    symbols = struct(); 
    symbols.q = q; 
    symbols.dq = dq; 
    symbols.ddq = ddq; 
    symbols.u = u;
   
    % Dynamics functions
    [H, C] = HandC(model, q, dq);
    % B = [SX.zeros(6,NA); SX.eye(NA)]; 
    ContinuousDynamics = H*ddq  + C - B * u; 
    obj.Dynamics = Function('ContinuousDynamics', {x, ddq, u},{ContinuousDynamics},{'x','ddq','u'},{'ContinuousDynamics'});  
    
    % Contact dynamics

    % Feet positions
    posvel = gcPosVel2D(model,q,dq); 
    contactdic = []; num = 1:NC; 
    for i = 1:NC
        grf{i} = SX.sym(append('grf_',model.gc.name{i}),2); % contact force variable
        foot{i} = posvel(1:2,i); % foot positions
        footFunc{i} = Function(append('footFunc_',model.gc.name{i}), {x}, {foot{i}},{'x'},{append('foot_',model.gc.name{i})}); 
        footspeed{i} = posvel(3:4,i); % foot speeds
        footvelFunc{i} = Function(append('FootVelFunc_',model.gc.name{i}), {x}, {footspeed{i}},{'x'},{append('foot_',model.gc.name{i})});
        J{i} = jacobian(foot{i}, q); % contact jacobian
        dJ{i} = jacobian(J{i}*dq,q); % derivative of contact jacobian
        contactdic = [contactdic, string(model.gc.name{i})]; 
    end

    % Add more casadi symbols
    symbols.grf = grf; 
    symbols.MassMatrix = H; 
    symbols.ode_sym = ContinuousDynamics; 
    obj.CasADi_Symbols = symbols; 

    % Contact information
    contact = struct(); 
    contact.Dictionary = dictionary(contactdic,num); 
    contact.FeetPositions = footFunc; 
    contact.FeetVelocities = footvelFunc; 
    contact.J = J; 
    contact.dJ = dJ; 
    obj.Contacts = contact; 

end 
