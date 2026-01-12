function phase_data = ExtractMultiphaseSolution2D(solution_vector, phase_config)
% Extract variables from multi-phase optimal control solution with finite elements
%
% Inputs:
%   solution_vector - Column vector containing all solution variables
%   phase_config - Structure array with fields:
%                  .n_dof - Number of degrees of freedom for this phase
%                  .n_elements - Number of finite elements for this phase
%                  .n_contacts - Number of active contacts for this phase
%
% Output:
%   phase_data - Structure array with extracted data for each phase

n_phases = length(phase_config);
phase_data = struct();

% Current position in solution vector
current_idx = 1;

for phase = 1:n_phases
    n_dof = phase_config(phase).n_dof;
    n_elements = phase_config(phase).n_points;
    n_contacts = phase_config(phase).n_contacts;
    n_input = n_dof - 2;  % Torque input vector is 6 rows less
    
    % Calculate total variables per finite element
    % Order: time + pos + vel + acc + torque_input + contact_forces
    vars_per_element = 1 + n_dof + n_dof + n_dof + n_input + (n_contacts * 2);
    
    % Total variables for this phase
    total_vars_phase = vars_per_element * n_elements;
    
    % Extract this phase's data
    phase_vector = solution_vector(current_idx:current_idx + total_vars_phase - 1);
    
    % Reshape to organize by finite elements (each column is one element)
    phase_matrix = reshape(phase_vector, vars_per_element, n_elements);
    
    % Extract individual variable types
    row_idx = 1;
    
    % Time vector (one per finite element)
    phase_data(phase).time = phase_matrix(row_idx, :)';
    row_idx = row_idx + 1;
    
    % Position
    phase_data(phase).position = phase_matrix(row_idx:row_idx + n_dof - 1, :)';
    row_idx = row_idx + n_dof;
    
    % Velocity
    phase_data(phase).velocity = phase_matrix(row_idx:row_idx + n_dof - 1, :)';
    row_idx = row_idx + n_dof;
    
    % Acceleration
    phase_data(phase).acceleration = phase_matrix(row_idx:row_idx + n_dof - 1, :)';
    row_idx = row_idx + n_dof;
    
    % Torque Input/Control
    phase_data(phase).torque = phase_matrix(row_idx:row_idx + n_input - 1, :)';
    row_idx = row_idx + n_input;
    
    % Ground Reaction Forces (each contact is a 3x1 vector)
    if n_contacts > 0
        contact_forces_raw = phase_matrix(row_idx:row_idx + (n_contacts * 2) - 1, :);
        % Reshape to organize contact forces: each contact gets 3 consecutive rows
        phase_data(phase).ground_reaction_forces = zeros(n_elements, 2, n_contacts);
        for contact = 1:n_contacts
            start_row = (contact - 1) * 2 + 1;
            end_row = contact * 2;
            phase_data(phase).ground_reaction_forces(:, :, contact) = contact_forces_raw(start_row:end_row, :)';
        end
    else
        phase_data(phase).ground_reaction_forces = [];
    end
    
    % Store configuration info
    phase_data(phase).n_dof = n_dof;
    phase_data(phase).n_elements = n_elements;
    phase_data(phase).n_input = n_input;
    phase_data(phase).n_contacts = n_contacts;
    
    % Update index for next phase
    current_idx = current_idx + total_vars_phase;
end

end