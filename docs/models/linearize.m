function  exp_out = linearize(exp_in, small_vars)

n_vars = length(small_vars);

exp_out = subs(exp_in,small_vars,zeros(1,n_vars));

for i = 1 : n_vars
    exp_out = exp_out + subs(diff(exp_in,small_vars{i}),small_vars,zeros(1,n_vars))*small_vars{i};
end

exp_out = simplify(exp_out);

end

