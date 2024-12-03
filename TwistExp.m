function g = TwistExp(xi)
    g = [hat(xi(4:6)), xi(1:3); zeros(1,3), 0];
end