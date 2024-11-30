function mu = manipulability(J, measure)
    % Input:
    %   J: body jacobian (6 × 6 matrix)
    %   measure: a single string argument that can only be one of ‘sigmamin’, ‘detjac’, or ‘invcond’.
    % Output:
    %   mu: The corresponding measure of manipulability
    assert(isequal(size(J), [6 6]), 'J must be a 6x6 matrix');
    singular_values = svd(J);
    sigma_min = singular_values(end);
    sigma_max = singular_values(1);

    if measure == "sigmamin"
        mu = sigma_min;
    elseif measure == "detjac"
        mu = det(J);
    elseif measure == "invcond"
        mu = sigma_min / sigma_max;
    else
        error("Incorrect command for manipulability")
    end
end