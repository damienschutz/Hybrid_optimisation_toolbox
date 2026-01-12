function refined_phase_data = RefineMultiphaseSolution(phase_data, refinement_factor)
    % REFINE_MULTIPHASE_SOLUTION Interpolates extracted multi-phase solution to finer mesh
    % Interpolates between rows (mesh points) to add intermediate nodes
    %
    % Inputs:
    %   phase_data: Structure array from extract_multiphase_solution_better
    %   refinement_factor: Factor by which to increase mesh density (e.g., 2 = double)
    %
    % Output:
    %   refined_phase_data: Structure array with same format but refined mesh
    
    n_phases = length(phase_data);
    refined_phase_data = struct();
    
    for phase = 1:n_phases
        fprintf('Refining phase %d...\n', phase);
        
        % Get original data
        current_phase = phase_data(phase);
        time_orig = current_phase.time;  % Column vector (n_elements x 1)
        [n_orig_rows, n_cols] = size(current_phase.position);  % n_elements x n_dof
        
        % Create index vector for original rows
        orig_indices = 1:n_orig_rows;
        
        % Create refined index vector (interpolate between rows)
        n_refined_rows = (n_orig_rows - 1) * refinement_factor + 1;
        refined_indices = linspace(1, n_orig_rows, n_refined_rows);
        
        % Initialize refined phase
        refined_phase_data(phase).n_dof = current_phase.n_dof;
        refined_phase_data(phase).n_elements = n_refined_rows;  % Updated element count
        refined_phase_data(phase).n_input = current_phase.n_input;
        refined_phase_data(phase).n_contacts = current_phase.n_contacts;
        
        % Interpolate time vector
        refined_phase_data(phase).time = interp1(orig_indices, time_orig, ...
            refined_indices, 'spline', 'extrap')';
        
        % Interpolate position (n_elements x n_dof -> n_refined_elements x n_dof)
        refined_phase_data(phase).position = zeros(n_refined_rows, current_phase.n_dof);
        for dof = 1:current_phase.n_dof
            refined_phase_data(phase).position(:, dof) = interp1(orig_indices, ...
                current_phase.position(:, dof), refined_indices, 'spline', 'extrap');
        end
        
        % Interpolate velocity (n_elements x n_dof)
        refined_phase_data(phase).velocity = zeros(n_refined_rows, current_phase.n_dof);
        for dof = 1:current_phase.n_dof
            refined_phase_data(phase).velocity(:, dof) = interp1(orig_indices, ...
                current_phase.velocity(:, dof), refined_indices, 'spline', 'extrap');
        end
        
        % Interpolate acceleration (n_elements x n_dof)
        refined_phase_data(phase).acceleration = zeros(n_refined_rows, current_phase.n_dof);
        for dof = 1:current_phase.n_dof
            refined_phase_data(phase).acceleration(:, dof) = interp1(orig_indices, ...
                current_phase.acceleration(:, dof), refined_indices, 'spline', 'extrap');
        end
        
        % Interpolate torque (n_elements x n_input)
        refined_phase_data(phase).torque = zeros(n_refined_rows, current_phase.n_input);
        for input = 1:current_phase.n_input
            refined_phase_data(phase).torque(:, input) = interp1(orig_indices, ...
                current_phase.torque(:, input), refined_indices, 'spline', 'extrap');
        end
        
        % Interpolate ground reaction forces (n_elements x 3 x n_contacts)
        if current_phase.n_contacts > 0
            refined_phase_data(phase).ground_reaction_forces = zeros(n_refined_rows, 3, current_phase.n_contacts);
            for contact = 1:current_phase.n_contacts
                for force_component = 1:3
                    % Extract the force component across all mesh points
                    force_data = current_phase.ground_reaction_forces(:, force_component, contact);
                    refined_phase_data(phase).ground_reaction_forces(:, force_component, contact) = ...
                        interp1(orig_indices, force_data, refined_indices, 'spline', 'extrap');
                end
            end
        else
            refined_phase_data(phase).ground_reaction_forces = [];
        end
    end
end
