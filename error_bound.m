function error_bound = error_bound(err, bound)
    if err > bound
        error_bound = bound;
    elseif err < -bound
        error_bound = -bound;
    else
        error_bound = err;
    end
end