clear all
clc
close all

folder = 'figs_final';

i_start = 2;
i_end = 3024;
found = false(1,i_end);
for i = i_start : i_end
    found(i) = isfile([folder,'/fig_',num2str(i),'.png']);
end

fprintf('[')
for id = find(~found(1:end))
    fprintf([num2str(id),', ']);
end
fprintf(']\n')

missing = find(~found(1:end));

new_chunk = true(1,length(missing));
for i = 2 : length(missing)
    if ( missing(i) == missing(i-1)+1 )
        new_chunk(i) = false;
    end
end

i_chunk_starts = find(new_chunk);

for i = 1:length(i_chunk_starts)-1
   chunk_start = missing(i_chunk_starts(i));
   chunk_end   = missing(i_chunk_starts(i+1)-1);
   fprintf(['export ISTART=', num2str(chunk_start), '; export IEND=', num2str(chunk_end),'; sbatch run.sh \n']);
end

chunk_start = missing(i_chunk_starts(end));
chunk_end   = missing(end);
fprintf(['export ISTART=', num2str(chunk_start), '; export IEND=', num2str(chunk_end),'; sbatch run.sh \n']);

