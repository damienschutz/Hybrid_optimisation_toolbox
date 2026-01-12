%% Function to define the CasADi symbolic constrainted dynamics

% Input is the structure containing the initial build of the dynamics for
% CasADi and a cell called contacts, which contains chracter strings that
% specify which points are in contact
% Output is the same structure with additional elements for the constrained
% dynamics and the holonomic constraints of the specified points in contact


% Created by Damien Schutz
% UCT
% 2025/04/23

function obj = AddContactDynamics(obj,contacts)
    import casadi.*
    
    dic = obj.Contacts.Dictionary;  
    contacts = string(contacts);

    title = ''; grf = []; J = []; dJ = [];
    for i = 1:length(contacts) 
       idx = dic(contacts(i)); 
       title = append(title,'_',contacts(i)); 
       grf = [grf;obj.CasADi_Symbols.grf{idx}];
       J = [J;obj.Contacts.J{idx}]; 
       dJ = [dJ;obj.Contacts.dJ{idx}]; 
    end
    
    title = char(title); 

    q = obj.CasADi_Symbols.q;
    dq = obj.CasADi_Symbols.dq; 
    ddq = obj.CasADi_Symbols.ddq; 
    u = obj.CasADi_Symbols.u; 
    x = [q;dq]; 

    ConDynamics = obj.CasADi_Symbols.ode_sym - J'*grf; 
    obj.ConstrainedDynamics = Function('ConstrainedDynamics', {x, ddq, u, grf}, {ConDynamics},{'x','ddq','u','grf'},{append('ConstrainedDynamics',title)}); 
    HolonomicConstraint = J*ddq + dJ*dq; 
    obj.Holonomic = Function('HolonomicConstraint', {x, ddq}, {HolonomicConstraint}, {'x','ddq'},{append('HolonomicConstraint',title)});
end