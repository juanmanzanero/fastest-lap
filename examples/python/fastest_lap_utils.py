import fastest_lap
import matplotlib.pyplot as plt
import numpy as np

def plot_turn(s_start,s_end,title):
    
    # (1) Download data
    s = fastest_lap.download_vector("track/arclength");
    x = fastest_lap.download_vector("track/centerline/x");
    y = -np.array(fastest_lap.download_vector("track/centerline/y"));
    x_left = fastest_lap.download_vector("track/left/x");
    y_left = -np.array(fastest_lap.download_vector("track/left/y"));
    x_right = fastest_lap.download_vector("track/right/x");
    y_right = -np.array(fastest_lap.download_vector("track/right/y"));
    x_left_measured = fastest_lap.download_vector("track/left_measured/x");
    y_left_measured = -np.array(fastest_lap.download_vector("track/left_measured/y"));
    x_right_measured = fastest_lap.download_vector("track/right_measured/x");
    y_right_measured = -np.array(fastest_lap.download_vector("track/right_measured/y"));
    yaw_dot = fastest_lap.download_vector("track/yaw_dot");
    nl = fastest_lap.download_vector("track/nl");
    nr = fastest_lap.download_vector("track/nr");    

    # (1.1) Get i_start and i_end
    s = np.array(s);
    i_start = (np.abs(s - s_start)).argmin()
    i_end = (np.abs(s - s_end)).argmin()
    
    # (2) Generate plots
    fig = plt.figure(constrained_layout=True, figsize=(15,11))
    gs = fig.add_gridspec(ncols=2, nrows=3, height_ratios=[4,1,1])
    ax1 = fig.add_subplot(gs[0, 0])
    ax3 = fig.add_subplot(gs[0, 1])    
    ax2 = fig.add_subplot(gs[1, :])    
    ax4 = fig.add_subplot(gs[2, :])    
    
    # (2.1) Zoomed GPS
    ax1.plot(x_left[i_start:i_end],y_left[i_start:i_end],'.w',visible='off');
    ax1.plot(x_right[i_start:i_end],y_right[i_start:i_end],'.w',visible='off');
    ax1.set_aspect('equal', adjustable='datalim')
    xlim=ax1.get_xlim(); ylim=ax1.get_ylim();
    ax1.plot(x,y,'.-w');
    ax1.plot(x[i_start:i_end],y[i_start:i_end],'.-',color='yellow',visible='off');
    ax1.plot(x_left,y_left,'.w');
    ax1.plot(x_right,y_right,'.w');
    ax1.plot(x_left_measured,y_left_measured,'y'); ax1.plot(x_right_measured,y_right_measured,'y'); 
    ax1.set_xlim(xlim); ax1.set_ylim(ylim)
    ax1.set_title('GPS',fontweight='bold',fontsize=32)
    
    # (2.2) Curvature
    ax2.plot(s[i_start:i_end],yaw_dot[i_start:i_end],'.-w');
    ax2.set_title('curvature',fontweight='bold',fontsize=32)
    
    # (2.3) Full view GPS
    ax3.plot(x,y,'w')
    ax3.plot(x[i_start:i_end],y[i_start:i_end],'yellow',linewidth=5)
    ax3.set_aspect('equal', adjustable='datalim')
    
    # (2.4) Distance to boundaries
    ax4.plot(s[i_start:i_end],nl[i_start:i_end],'.-'); ax4.plot(s[i_start:i_end],nr[i_start:i_end],'.-');
    ax4.set_title('distance to boundaries',fontweight='bold',fontsize=32)
    fig.suptitle(title, fontsize=32, fontweight='bold', fontname='DejaVu Sans')
