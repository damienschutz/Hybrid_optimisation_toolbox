function var = AddVariable(var, name, size, lb, ub, init)
    import casadi.*
    sym = SX.sym(char(name), size); 
    var.Name = name; 
    var.Symbolic = sym; 
    var.Size = size; 
    var.LowerBound = lb; 
    var.UpperBound = ub; 
    var.Init = zeros(size); 
    if nargin == 6
        var.Init = init; 
    end
end