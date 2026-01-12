 function J = Cost(type, u, x)
    import casadi.*
            if type == "Sum Torques"
                J = u'*u; 
            elseif type == "Mechanical Work"
                idx = length(x)/2+7; 
                dq_u = x{idx:end}; 
                E = u'*dq_u; 
                
                J = E; 
                J = fmax(0,E);

            else
                J = 0;
            end
end