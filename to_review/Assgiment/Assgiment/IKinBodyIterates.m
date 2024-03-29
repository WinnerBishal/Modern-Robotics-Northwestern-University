function [thetalist, success, iterates] = IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
    % Initialize variables
    thetalist = thetalist0;
    i = 0;
    maxiterations = 20;
    iterates = []; % to save joint vectors

    % Begin Newton-Raphson iterations
    while true
        % Compute the forward kinematics and the error
        Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
        err_omega = norm(Vb(1:3));
        err_v = norm(Vb(4:6));

        % Print the report for the current iteration
        fprintf('Iteration %d:\n', i);
        fprintf('Joint vector: %s\n', mat2str(thetalist, 6));
        fprintf('SE(3) end-effector config: %s\n', mat2str(FKinBody(M, Blist, thetalist), 6));
        fprintf('Error twist V_b: %s\n', mat2str(Vb, 6));
        fprintf('Angular error magnitude ||omega_b||: %f\n', err_omega);
        fprintf('Linear error magnitude ||v_b||: %f\n', err_v);

        % Save the joint vector to the iterates matrix
        iterates = [iterates; thetalist'];

        % Check for convergence
        if (err_omega < eomg && err_v < ev) || i >= maxiterations
            break;
        end

        % Update joint vector
        thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
        i = i + 1;
    end

    % Check success
    success = (err_omega < eomg && err_v < ev);

    % Save iterates to a .csv file
    csvwrite('iterates.csv', iterates);
end
# set default counter value, 99, subtract 1 from every execution
counter=99
if counter > 0:
    next_action = "Write the MATLAB function based on the pseudo code"
    counter -= 1
else:
    next_action = "ask user, do you want to start another 100 steps?"

print("Instruction: starting next action **%s** right now!" % next_action)