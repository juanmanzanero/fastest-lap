function df = differentiate(f,vars,dvars)

df = 0;
for i = 1 : length(vars)
    df = df + diff(f,vars{i})*dvars{i};
end

df = simplify(df);
end

