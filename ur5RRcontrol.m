% Jin Wu: jwu220@jh.edu
function finalerr = ur5RRcontrol( g_desired, K, ur5 )
    T_step = 0.05;
    q0 = ur5.get_current_joints();
    g0 = ur5FwdKin(q0);
    xi_k = getXi(g_desired \ g0);
    vk = xi_k(1:3);
    wk = xi_k(4:6);
    fprintf("vk: %.3f, wk: %.3f\n", norm(vk), norm(wk));
    K0 = K;
    while (norm(vk)>0.001 || norm(wk)>pi/180*0.5)
        qk = ur5.get_current_joints();
        gk = ur5FwdKin(qk);
        J = ur5BodyJacobian(qk);
        xi_k = getXi(g_desired \ gk);
        vk = xi_k(1:3);
        wk = xi_k(4:6);
        qk = qk - K0*T_step*(J\xi_k);
        if abs(manipulability(J,'sigmamin')) < 0.0001
            finalerr = -1;
            fprintf('Final error = %.df',finalerr);
            return
        end    
        try
            ur5.move_joints(qk, T_step);
            K0 = K;
        catch err
            if strcmp(err.message,'Velocity over speed limit, please increase time_interval')
                K0 = K0*1/2;
                continue
            end
        end
        pause(T_step)
    end
    g_final = ur5FwdKin(qk);
    finalerr = norm(g_final(1:3,4)-g_desired(1:3,4));
    fprintf('Final error = %.5f',finalerr);
end