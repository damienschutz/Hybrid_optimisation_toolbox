function seed = CreateNewSeed(refined_sol)
    X = []; 
    n_elements = refined_sol(1).n_elements;
    for i=1:length(refined_sol)
        for j = 1:n_elements
            X = [X;refined_sol(i).time(j,:)];
            X = [X;refined_sol(i).position(j,:)'];
            X = [X;refined_sol(i).velocity(j,:)'];
            X = [X;refined_sol(i).acceleration(j,:)'];
            X = [X;refined_sol(i).torque(j,:)'];

            if refined_sol(i).n_contacts > 0
                grf = refined_sol(i).ground_reaction_forces(j,:,:);
                for k = 1:refined_sol(i).n_contacts
                    X = [X;grf(:,:,k)'];
                end
            end
        end
    end
    seed = X; 
end