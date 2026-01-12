function obj = FrictionConstraint(obj, F, frictiontype)
    import casadi.*
    mu = 1.3;
    counter = 1; 

    switch frictiontype
        case 'Cone'
            for i = 1:numel(F)/3
            Fc = F(counter:counter+2);
            obj.ConstraintList = obj.ConstraintList.AddConstraint('Friction Cone',norm_2([Fc(1); Fc(2)]) - mu*Fc(3), -inf,0);
            obj.ConstraintList = obj.ConstraintList.AddConstraint('Normal Friction Force',Fc(3),0,inf);
            counter = counter+3; 
            end


        case 'Pyramid'
            for i = 1:numel(F)/3
            Fc = F(counter:counter+2);
            sym_con = [Fc(1) - 1/sqrt(2)*mu*Fc(3); 
                                -Fc(1) - 1/sqrt(2)*mu*Fc(3);
                                Fc(2) - 1/sqrt(2)*mu*Fc(3);
                                -Fc(2) - 1/sqrt(2)*mu*Fc(3)];
    
            obj.ConstraintList = obj.ConstraintList.AddConstraint('Friction Pyramid',sym_con, -inf*ones(4,1),zeros(4,1));
            obj.ConstraintList = obj.ConstraintList.AddConstraint('Normal Friction Force',Fc(3),0,inf);
            counter = counter+3; 
            end
    end

end