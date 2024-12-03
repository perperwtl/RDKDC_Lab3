function Ad = RigidAdjoint(g)

    R = g(1:3, 1:3);  % Rotation part
    p = g(1:3, 4);    % Translation part
    Ad = [R, hat(p) * R;
          zeros(3), R];
end
