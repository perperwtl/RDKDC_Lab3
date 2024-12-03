function xi = RevoluteTwist(q, w)
    v = -cross(w, q);
    xi = [v; w];
end