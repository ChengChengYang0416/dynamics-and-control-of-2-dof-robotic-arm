function error_bound = error_bound(err)
    bound = 5;
    if err > bound
        error_bound = bound;
    elseif err < -bound
        error_bound = -bound;
    else
        error_bound = err;
    end
end