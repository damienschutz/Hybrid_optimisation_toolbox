classdef Variable
    properties
        Name
        Type
        Symbolic
        Size
        LowerBound
        UpperBound
        Init
    end
    methods
        function obj = Variable(name, size, varargin)
            import casadi.*
            sym = MX.sym(char(name), size);  
            obj.Symbolic = sym; 
            obj.Size = size; 
            obj.LowerBound = -inf*ones(size); 
            obj.UpperBound = inf*ones(size); 
            obj.Init = zeros(size); 
            obj.Name = name; 
            if length(varargin) >= 1
                obj.LowerBound = varargin{1}; 
            end
            if length(varargin) >= 2
                obj.UpperBound = varargin{2}; 
            end
            if length(varargin) >= 3
                obj.Init = varargin{3}; 
            end
            if length(varargin) >= 4
                obj.Type = varargin{4}; 
            end
            if length(varargin) >= 5
                disp('Too many input arguments. Maximum is 6') 
            end
        end

        function obj = AddBounds(obj,lb,ub)
            if isempty(lb)
            else
                obj.LowerBound = lb; 
                obj.UpperBound = ub;
            end
        end
    end
end