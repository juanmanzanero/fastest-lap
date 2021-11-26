loadlibrary('/Users/juanmanzanero/Documents/software/fastest-lap/build/lib/libfastestlapc.dylib','/Users/juanmanzanero/Documents/software/fastest-lap/src/main/c/fastestlapc.h');

vehicle = calllib('libfastestlapc','create_vehicle',[],'car3','lot 2016 kart','cartesian','/Users/juanmanzanero/Documents/software/fastest-lap/database/roberto-lot-kart-2016.xml');

n_points = 100;
ax = zeros(1,n_points);
ay_min = zeros(1,n_points);
ay_max = zeros(1,n_points);

[ay,ax_max,ax_min] = calllib('libfastestlapc','gg_diagram',ax,ay_max,ay_min,vehicle,120/3.6,n_points);

unloadlibrary libfastestlapc;