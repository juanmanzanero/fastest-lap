# Compute GG diagram
import matplotlib.pyplot as plt
import ctypes as c
import numpy as np
import pathlib
import xml.etree.ElementTree as xml

KMH=1.0/3.6;

libname="${libdir_python}/$<TARGET_FILE_NAME:fastestlapc>"
c_lib = c.CDLL(libname)
c_lib.download_scalar.restype       = c.c_double;
c_lib.track_download_length.restype = c.c_double;
c_lib.vehicle_type_get_sizes.argtypes = [c.POINTER(c.c_int), c.POINTER(c.c_int), c.POINTER(c.c_int), c.POINTER(c.c_int), c.c_char_p];

# Print -----------------------------------------------------------------------------

def set_print_level(print_level):
	c_lib.set_print_level(c.c_int(print_level));
	return;

def print_variables():
	c_lib.print_variables()
	return;

def print_variable(variable_name):
	variable_name = c.c_char_p((variable_name).encode('utf-8'));
	
	n_char = pow(2,20);
	c_data = (c.c_char*n_char)();
	c_lib.print_variable_to_string(c_data, c.c_int(n_char), variable_name);
	print(c_data.value.decode())
	return;

# Factories -------------------------------------------------------------------------

def create_vehicle_from_xml(name,database_file):
	name = c.c_char_p((name).encode('utf-8'))
	database_file = c.c_char_p((database_file).encode('utf-8'))

	c_lib.create_vehicle_from_xml(name,database_file)

	return;

def create_vehicle_empty(name,vehicle_type):
	name = c.c_char_p((name).encode('utf-8'))
	vehicle_type = c.c_char_p((vehicle_type).encode('utf-8'))

	c_lib.create_vehicle_empty(name,vehicle_type)

	return;

def create_track_from_xml(name,track_file):
	c_name = c.c_char_p((name).encode('utf-8'));
	c_track_file = c.c_char_p((track_file).encode('utf-8'));

	c_lib.create_track_from_xml(c_name,c_track_file);

	return;

def create_vector(name, data):
	name = c.c_char_p((name).encode('utf-8'))
	n = len(data)
	c_data = (c.c_double*n)();

	for i in range(n):
		c_data[i] = data[i];

	c_lib.create_vector(name, c.c_int(n), c_data);
	return;

def create_scalar(name, data):
	name = c.c_char_p((name).encode('utf-8'))
	c_lib.create_scalar(name, c.c_double(data));
	return;

def copy_variable(old_name, new_name):
	old_name = c.c_char_p((old_name).encode('utf-8'))
	new_name = c.c_char_p((new_name).encode('utf-8'))

	c_lib.copy_variable(old_name, new_name);
	return;

def move_variable(old_name, new_name):
	old_name = c.c_char_p((old_name).encode('utf-8'))
	new_name = c.c_char_p((new_name).encode('utf-8'))

	c_lib.move_variable(old_name, new_name);
	return;


# Destructors ---------------------------------------------------------------------------------------------------------

def delete_variable(name):
	name = c.c_char_p((name).encode('utf-8'))
	c_lib.delete_variable(name);

# Getters --------------------------------------------------------------

def variable_type(name):
	c_variable = c.c_char_p((name).encode('utf-8'))
	str_len_max = 99;
	c_variable_type = c.c_char_p(((" ")*str_len_max).encode('utf-8'));

	c_lib.variable_type(c_variable_type, str_len_max, c_variable);

	return c_variable_type.value.decode()

def download_variables(prefix,variable_list):
	varmap = dict()
	for var in variable_list:
		if ( variable_type(prefix+var) == 'scalar' ):
			varmap[var] = download_scalar(prefix+var)

		elif ( variable_type(prefix+var) == 'vector' ):
			varmap[var] = download_vector(prefix+var)
	
		else:
			raise Exception

	return varmap

def download_scalar(name):
	c_variable = c.c_char_p((name).encode('utf-8'))	
	return c_lib.download_scalar(c_variable)

def download_vector_size(name):
	c_variable = c.c_char_p((name).encode('utf-8'))	
	return c_lib.download_vector_size(c_variable);

def download_vector(name):
	c_variable = c.c_char_p((name).encode('utf-8'))	
	n = c_lib.download_vector_size(c_variable)

	c_data = (c.c_double*n)();
	c_lib.download_vector(c_data, c.c_int(n), c_variable);
	data = [None]*n;
	for i in range(n):
		data[i] = c_data[i];

	return data;

def vehicle_type_get_sizes(vehicle_type_name):
	c_vehicle_type_name = c.c_char_p((vehicle_type_name).encode('utf-8'));
	
	c_n_state     = (c.c_int*1)();
	c_n_algebraic = (c.c_int*1)();
	c_n_control   = (c.c_int*1)();
	c_n_outputs   = (c.c_int*1)();

	c_lib.vehicle_type_get_sizes(c_n_state, c_n_algebraic, c_n_control, c_n_outputs, c_vehicle_type_name);

	return c_n_state[0], c_n_algebraic[0], c_n_control[0], c_n_outputs[0]
	

def vehicle_type_get_names(vehicle_type_name):
	c_vehicle_type_name = c.c_char_p((vehicle_type_name).encode('utf-8'))
	n_state,n_algebraic,n_control,n_outputs = vehicle_type_get_sizes(vehicle_type_name);
	string_size = 99;
	
	c_key_name = c.c_char_p( (" "*string_size).encode('utf-8') );

	state_names = [None]*n_state;
	for i in range(n_state):
		state_names[i] = (" "*string_size).encode('utf-8');
	
	algebraic_names = [None]*n_algebraic;
	for i in range(n_algebraic):
		algebraic_names[i] = (" "*string_size).encode('utf-8');

	control_names = [None]*n_control;
	for i in range(n_control):
		control_names[i] = (" "*string_size).encode('utf-8');
	
	output_names = [None]*n_outputs;
	for i in range(n_outputs):
		output_names[i] = (" "*string_size).encode('utf-8');
	
	c_state_names = (c.c_char_p*n_state)();
	c_state_names[:] = state_names;

	c_algebraic_names = (c.c_char_p*n_algebraic)();
	c_algebraic_names[:] = algebraic_names;

	c_control_names = (c.c_char_p*n_control)();
	c_control_names[:] = control_names;

	c_output_names = (c.c_char_p*n_outputs)();
	c_output_names[:] = output_names;

	c_lib.vehicle_type_get_names(c_key_name, c_state_names, c_algebraic_names, c_control_names, c_output_names, c.c_int(string_size), c_vehicle_type_name)

	for i in range(n_state):
		state_names[i] = c_state_names[i].decode();

	for i in range(n_algebraic):
		algebraic_names[i] = c_algebraic_names[i].decode();

	for i in range(n_control):
		control_names[i] = c_control_names[i].decode();

	for i in range(n_outputs):
		output_names[i] = c_output_names[i].decode();

	return c_key_name.value.decode(), state_names, algebraic_names, control_names, output_names;

def vehicle_get_output(vehicle_name, q, qa, u, s, property_name):
	c_vehicle_name = c.c_char_p((vehicle_name).encode('utf-8'))	
	c_property_name = c.c_char_p((property_name).encode('utf-8'))	
	
	c_q = (c.c_double*len(q))();
	for i in range(len(q)):
		c_q[i] = q[i];

	c_qa = (c.c_double*len(qa))();
	for i in range(len(qa)):
		c_qa[i] = qa[i];

	c_u = (c.c_double*len(u))();
	for i in range(len(u)):
		c_u[i] = u[i];

	c_lib.vehicle_get_output(c_vehicle_name, c_q, c_qa, c_u, c.c_double(s), c_property_name);
	return;

def vehicle_save_as_xml(vehicle_name, xml_file_name):
	c_vehicle_name = c.c_char_p((vehicle_name).encode('utf-8'));
	c_xml_file_name = c.c_char_p((xml_file_name).encode('utf-8'));

	c_lib.vehicle_save_as_xml(c_vehicle_name, c_xml_file_name);
	return;

def track_download_length(track_name):
	c_track_name = c.c_char_p((track_name).encode('utf-8'));
	return c_lib.track_download_length(c_track_name);

def track_download_data(track_name, variable_name):
	c_track_name = c.c_char_p((track_name).encode('utf-8'));
	c_variable_name = c.c_char_p((variable_name).encode('utf-8'));

	n_points = c_lib.track_download_number_of_points(c_track_name);
	c_data = (c.c_double*n_points)();
	c_lib.track_download_data(c_data, c_track_name, c.c_int(n_points), c_variable_name);

	data = [None]*n_points;
	for i in range(n_points):
		data[i] = c_data[i];

	return data;



# Modifiers ---------------------------------------------------------------------------


def vehicle_set_parameter(vehicle,parameter_name,parameter_value):
	vehicle = c.c_char_p((vehicle).encode('utf-8'))
	parameter_name = c.c_char_p((parameter_name).encode('utf-8'));
	c_lib.vehicle_set_parameter(vehicle,parameter_name,c.c_double(parameter_value))
	return ;

def vehicle_declare_new_constant_parameter(vehicle_name, parameter_path, parameter_alias, parameter_value):
	c_vehicle_name = c.c_char_p((vehicle_name).encode('utf-8'));
	c_parameter_path = c.c_char_p((parameter_path).encode('utf-8'));
	c_parameter_alias = c.c_char_p((parameter_alias).encode('utf-8'));

	c_lib.vehicle_declare_new_constant_parameter(c_vehicle_name, c_parameter_path, c_parameter_alias, c.c_double(parameter_value));
	return;

def vehicle_change_track(vehicle_name, track_name):
	c_vehicle_name = c.c_char_p((vehicle_name).encode('utf-8'));
	c_track_name = c.c_char_p((track_name).encode('utf-8'));
	
	c_lib.vehicle_change_track(c_vehicle_name, c_track_name);
	return;

# Applications ------------------------------------------------------------------------

def circuit_preprocessor(options):
	c_options = c.c_char_p((options).encode('utf-8'));
	c_lib.circuit_preprocessor(c_options);
	
	return;
def gg_diagram(vehicle,speed,n_points):
	vehicle = c.c_char_p((vehicle).encode('utf-8'))
	ay_c = (c.c_double*n_points)();
	ax_max_c = (c.c_double*n_points)();
	ax_min_c = (c.c_double*n_points)();
	c_lib.gg_diagram(ay_c, ax_max_c, ax_min_c, vehicle, c.c_double(speed), c.c_int(n_points));

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

def optimal_laptime(vehicle, track, s, options):
	c_vehicle = c.c_char_p((vehicle).encode('utf-8'))
	track   = c.c_char_p((track).encode('utf-8'))

	# Get channels ready to be written by C++
	c_s = (c.c_double*len(s))();

	for i in range(len(s)):
		c_s[i] = s[i];

	c_options = c.c_char_p((options).encode('utf-8'));

	c_lib.optimal_laptime(c_vehicle, track, c.c_int(len(s)), c_s, c_options);

	# Parse the options to get the variable names
	root = xml.fromstring(options)

	variable_list = []

	if ( root.find('output_variables') != None ):
		# Find prefix
		prefix_element = root.find('output_variables').find('prefix');
		if ( prefix_element != None ):
			prefix = prefix_element.text;
		else:
			prefix = '';

		# Find variables
		variables_element = root.find('output_variables').find('variables');

		if ( variables_element != None ):
			for var in variables_element.find('*'):
				variables_list.append(var.tag);	

		else:
			key_name, state_names, algebraic_names, control_names, outputs_names = vehicle_type_get_names(variable_type(vehicle))
			variable_list = [key_name] + state_names + algebraic_names + control_names + outputs_names;

	return prefix.strip(),variable_list;

def track_coordinates(track):
	x_center =  np.array(track_download_data(track,"centerline.x"));
	y_center = -np.array(track_download_data(track,"centerline.y"));
	x_left   =  np.array(track_download_data(track,"left.x"));
	y_left   = -np.array(track_download_data(track,"left.y"));
	x_right  =  np.array(track_download_data(track,"right.x"));
	y_right  = -np.array(track_download_data(track,"right.y"));
	theta    = -np.array(track_download_data(track,"heading-angle"));

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
	fig = plot_track(*track_coordinates(track))
	plt.plot(x,y,linewidth=2,color="orange");
