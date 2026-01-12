function obj = AddImpact2D(obj, contacts)
    import casadi.*
    
    dic = obj.Contacts.Dictionary;  
    contacts = string(contacts);

    title = ''; J = [];
    for i = 1:length(contacts) 
       idx = dic(contacts(i)); 
       title = append(title,'_',contacts(i)); 
       J = [J;obj.Contacts.J{idx}]; 
    end

    q = obj.CasADi_Symbols.q;
    dq = obj.CasADi_Symbols.dq; 
    x = [q;dq];
    M = obj.CasADi_Symbols.MassMatrix; 
    NB = length(q); 
    NI = 2*length(contacts);

    LHS = [M, J'; J, SX(NI, NI)]; 
    RHS = [M * dq; SX(NI, 1)]; 

    res = LHS\RHS; 
    dq_post_impact = res(1:NB); 
    title = char(title); 
    obj.Impact = Function(append('Impact',title), {x}, {[q;dq_post_impact]}, {'x_pre'},{'x_post'}); 
    
end