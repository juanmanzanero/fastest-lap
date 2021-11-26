function nabla_f = grad(f,x,x0)

nabla_f = sym(zeros(1,length(x)));

for i = 1 : length(x)
    nabla_f(i) = subs(diff(f,x{i}),x,x0);
end
end

