# Compute GG diagram
import matplotlib.pyplot as plt
import ctypes as c
import numpy as np
import pathlib

KMH=1.0/3.6;

libname=str(pathlib.Path(__file__).parent.absolute()) + "/" + "${libdir_python}/$<TARGET_FILE_NAME:fastestlapc>"
c_lib = c.CDLL(libname)

class c_Vehicle(c.Structure):
    _fields_ = [("name", c.c_char_p),
                ("type", c.c_int),
                 ("database_file", c.c_char_p),
               ]

class c_Track(c.Structure):
    _fields_ = [("name", c.c_char_p),
                ("track_file", c.c_char_p),
                ("is_closed", c.c_bool)
               ]

def load_vehicle(name,vehicle_type,database_file):
	name = c.c_char_p((name).encode('utf-8'))
	database_file = c.c_char_p((database_file).encode('utf-8'))
	vehicle_type = c.c_char_p((vehicle_type).encode('utf-8'))

	vehicle = c_Vehicle()
	c_lib.create_vehicle(c.byref(vehicle),name,vehicle_type,database_file)

	return vehicle;

def load_track(track_file,name):
	options="<options> <save_variables> <prefix>track/</prefix> <variables> <s/> </variables> </save_variables> </options>";
	c_name = c.c_char_p((name).encode('utf-8'));
	c_track_file = c.c_char_p((track_file).encode('utf-8'));
	c_options = c.c_char_p((options).encode('utf-8'));

	track = c_Track();
	c_lib.create_track(c.byref(track),c_name,c_track_file,c_options);

	# Get the results
	c_variable = c.c_char_p(("track/s").encode('utf-8'));
	n_points = c_lib.download_vector_table_variable_size(c_variable);
	c_s = (c.c_double*n_points)();
	c_lib.download_vector_table_variable(c_s, c.c_int(n_points), c_variable);
	s = [None]*n_points;
	for i in range(n_points):
		s[i] = c_s[i];

	# Clean up
	c_lib.clear_tables_by_prefix(c.c_char_p(("track/").encode('utf-8')));

	return track,s;

def set_scalar_parameter(vehicle,parameter_name,parameter_value):
	parameter_name = c.c_char_p((parameter_name).encode('utf-8'));
	c_lib.set_scalar_parameter(c.byref(vehicle),parameter_name,c.c_double(parameter_value))
	return vehicle;

def set_vector_parameter(vehicle,parameter_name,parameter_value):
	parameter_name = c.c_char_p((parameter_name).encode('utf-8'));
	c_parameter_value = (c.c_double*3)(parameter_value[0],parameter_value[1],parameter_value[2]);
	c_lib.set_vector_parameter(c.byref(vehicle),parameter_name,c_parameter_value)
	return vehicle;

def set_matrix_parameter(vehicle,parameter_name,parameter_value):
	parameter_name = c.c_char_p((parameter_name).encode('utf-8'));
	c_parameter_value = (c.c_double*9)(parameter_value[0],parameter_value[1],parameter_value[2],  
	                                   parameter_value[3],parameter_value[4],parameter_value[5],  
	                                   parameter_value[6],parameter_value[7],parameter_value[8]); 
	c_lib.set_matrix_parameter(c.byref(vehicle),parameter_name,c_parameter_value)
	return vehicle;

def gg_diagram(vehicle,speed,n_points):
	ay_c = (c.c_double*n_points)();
	ax_max_c = (c.c_double*n_points)();
	ax_min_c = (c.c_double*n_points)();
	c_lib.gg_diagram(ay_c, ax_max_c, ax_min_c, c.byref(vehicle), c.c_double(speed), c.c_int(n_points));

	ay = [None] * n_points;
	ay_minus = [None] * n_points;
	ax_max = [None] * n_points;
	ax_min = [None] * n_points;

	for i in range(n_points):
        	ay[i] = ay_c[i]/9.81;
        	ay_minus[i] = -ay[i];        
        	ax_max[i] = ax_max_c[i]/9.81;
        	ax_min[i] = ax_min_c[i]/9.81;

	return ay,ay_minus,ax_max,ax_min;

def optimal_laptime(vehicle, track, s, channels):

	# Get channels ready to be written by C++
	n_channels = len(channels);
	c_channels_name = ((c.c_char_p)*n_channels)();
	c_channels_data = (c.POINTER(c.c_double)*n_channels)()
	c_s = (c.c_double*len(s))();

	for i in range(len(s)):
		c_s[i] = s[i];

	options = "<options> <save_variables> <prefix>run/</prefix> <variables>";

	for channel in channels:
		options += "<" + channel + "/> ";

	options += "</variables> </save_variables> </options>";
	
	c_options = c.c_char_p((options).encode('utf-8'));

	c_lib.optimal_laptime(c.byref(vehicle), c.byref(track), c.c_int(len(s)), c_s, c_options);

	# Get the results
	result = dict();
	for channel in channels:
		c_data = (c.c_double*len(s))();
		c_variable = c.c_char_p(("run/" + channel).encode('utf-8'));
		c_lib.download_vector_table_variable(c_data, c.c_int(len(s)), c_variable);
		data = [None]*len(s);
		for i in range(len(s)):
			data[i] = c_data[i];
		result[channel] = data;	

	# Clean up
	c_lib.clear_tables_by_prefix(c.c_char_p(("run/").encode('utf-8')));

	return result;

def track_coordinates(track,s):
	n_points = len(s);

	c_x_center = (c.c_double*n_points)();
	c_y_center = (c.c_double*n_points)();
	c_x_left   = (c.c_double*n_points)();
	c_y_left   = (c.c_double*n_points)();
	c_x_right  = (c.c_double*n_points)();
	c_y_right  = (c.c_double*n_points)();
	c_theta    = (c.c_double*n_points)();
	c_s = (c.c_double*len(s))();

	for i in range(len(s)):
		c_s[i] = s[i];

	c_lib.track_coordinates(c_x_center, c_y_center, c_x_left, c_y_left, c_x_right, c_y_right, c_theta, c.byref(track), c.c_int(n_points), c_s);

	x_center = [None] * n_points;
	y_center = [None] * n_points;
	x_left   = [None] * n_points;
	y_left   = [None] * n_points;
	x_right  = [None] * n_points;
	y_right  = [None] * n_points;
	theta    = [None] * n_points;

	for i in range(n_points):
        	x_center[i] = c_x_center[i];
        	y_center[i] = c_y_center[i];
        	x_left[i]   = c_x_left[i];
        	y_left[i]   = c_y_left[i];
        	x_right[i]  = c_x_right[i];
        	y_right[i]  = c_y_right[i];
        	theta[i]    = c_theta[i];   

	return x_center, y_center, x_left, y_left, x_right, y_right, theta;

def plot_gg(ay,ay_minus,ax_max,ax_min):
	# initializing the figure
	fig = plt.figure()
	rect = [0, 0,2.0, 2.0]
	
	ax_polar = fig.add_axes([0.25,0.25,1.5,1.5], polar=True, frameon=False)
	ax_polar.grid(True,zorder=0)
	
	ax_carthesian  = fig.add_axes(rect,zorder=1)
	ax_carthesian.patch.set_alpha(0.0)
	plt.xlabel('$a_x$ [g]',fontsize=30)
	plt.ylabel('$a_y$ [g]',fontsize=30)
	ax_carthesian.set_aspect('equal', adjustable='box')
	plt.xlim(-2,2)
	plt.ylim(-2,2)
	# the polar axis:
	
	
	# plotting the line on the carthesian axis
	#ax_carthesian.plot(line,'b')
	
	# the polar plot
	#ax_polar.plot(theta, r, color='w', linewidth=3)
	ax_polar.set_rmax(1.5)
	ax_polar.set_rlim(0.0)
	ax_polar.set_rticks([0.5,1.0,1.5])
	
	
	tick = [ax_polar.get_rmax(),ax_polar.get_rmax()*0.97]
	for t  in np.deg2rad(np.arange(0,360,5)):
	    ax_polar.plot([t,t], tick, lw=0.72, color='#546379')
	
	ax_carthesian.grid(False)
	#circle = pl.Circle((0.0, 0.0), 1.5, transform=ax_polar.transData._b, edgecolor='#546379', alpha=0.1, linewidth=20)
	c = plt.Circle((0,0), 1.5, fill=False, edgecolor=(0.5372549019607843, 0.6039215686274509, 0.7215686274509804, 1.0), alpha=None,linewidth=3.0)
	ax_carthesian.add_artist(c)
	#ax_carthesian.plot([-2,2],[0,1],linewidth=14)
	# set the size of theta ticklabels (works)
	thetatick_locs = np.linspace(0.,315,8)
	thetatick_labels = []
	ax_polar.set_thetagrids(thetatick_locs, thetatick_labels, fontsize=16)
	
	# set the size of r ticklabels (does not work)
	rtick_locs = [0.5,1,1.5]
	ax_polar.set_rgrids(rtick_locs, [], fontsize=16)
	
	ax_carthesian.plot(ay,ax_max,color="yellow")
	ax_carthesian.plot(ay,ax_min,color="yellow")
	ax_carthesian.plot(ay_minus,ax_max,color="yellow")
	ax_carthesian.plot(ay_minus,ax_min,color="yellow")
	plt.show()

	return fig;

def plot_track(x_center, y_center, x_left, y_left, x_right, y_right, theta):
	fig = plt.figure(figsize=(14,7));
	plt.axis('equal');
	plt.grid(False);
	plt.plot(x_center,y_center,linewidth=0.5,color=(0.5372549019607843, 0.6039215686274509, 0.7215686274509804, 1.0),linestyle=(0, (20, 4)));
	plt.plot(x_left,y_left,linewidth=1,color=(0.5372549019607843, 0.6039215686274509, 0.7215686274509804, 1.0));
	plt.plot(x_right,y_right,linewidth=1,color=(0.5372549019607843, 0.6039215686274509, 0.7215686274509804, 1.0));

	return fig;

def plot_optimal_laptime(s, x, y, track):
	fig = plot_track(*track_coordinates(track,s))
	plt.plot(x,y,linewidth=2,color="orange");
