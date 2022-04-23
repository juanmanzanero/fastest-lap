function corners_stats = preprocess_telemetry(s,time,speed,speed_max,delta_speed)
    % (0) Set lsqnonlin options
    opts = optimoptions('lsqnonlin','Display','off');
    % (1) Get the local minima corresponding to corners below speed_max
    is_corner = islocalmin(speed) & (speed < speed_max);
    idx_corners = find(is_corner);
    n_corners = length(idx_corners);
    
    % (2) For each corner, find the velocity of its associated braking
    % point
    idx_braking = zeros(1,n_corners);
    
    for i_corner = 1 : n_corners
       i_point = idx_corners(i_corner);
       is_found = false;
       
       while (~is_found)
          if (i_point == 1)
              error('Braking point was not found');
          end
          if ( speed(i_point-1) < speed(i_point) )
            is_found = true;
            idx_braking(i_corner) = i_point;
          end
          i_point = i_point - 1;
       end
       
    end
    
    % (3) For each corner, find the time to rise to accelerate up to delta_speed
    t_rise_accelerate = zeros(1,n_corners);
    for i_corner = 1 : n_corners-1
        f = @(t)(interp1(time(idx_corners(i_corner):idx_braking(i_corner+1)),speed(idx_corners(i_corner):idx_braking(i_corner+1)),t,'spline')-speed(idx_corners(i_corner))-delta_speed);
        t_rise_accelerate(i_corner) = lsqnonlin(f,time(idx_corners(i_corner)+2),time(idx_corners(i_corner)), time(idx_braking(i_corner+1)), opts)-time(idx_corners(i_corner));       
    end
    
    % (3.1) Do the last corner
    f = @(t)(interp1(time(idx_corners(end):end),speed(idx_corners(end):end),t)-speed(idx_corners(end))-delta_speed);
    t_rise_accelerate(end) = lsqnonlin(f,time(idx_corners(end)+2),time(idx_corners(end)), time(end), opts)-time(idx_corners(end));           
    
    
    % (3) For each corner, find the time to rise to while braking to delta_speed
    t_rise_brake = zeros(1,n_corners);
    for i_corner = 1 : n_corners
        f = @(t)(interp1(time(idx_braking(i_corner):idx_corners(i_corner)),speed(idx_braking(i_corner):idx_corners(i_corner)),t,'spline')-speed(idx_corners(i_corner))-delta_speed);
        t_rise_brake(i_corner) = lsqnonlin(f,time(idx_corners(i_corner)-2),time(idx_braking(i_corner)), time(idx_corners(i_corner)), opts)-time(idx_corners(i_corner));       
    end
    
%     plot(time,speed,'.-');
%     hold on
%     plot(time(idx_corners),speed(idx_corners),'o');
%     plot(time(idx_braking),speed(idx_braking),'o');
%     plot(time(idx_corners)+t_rise_accelerate, interp1(time,speed,time(idx_corners)+t_rise_accelerate),'o');
%     plot(time(idx_corners)+t_rise_brake, interp1(time,speed,time(idx_corners)+t_rise_brake),'o');
    
    % (4) Output
    corners_stats = cell(1,n_corners);
    
    for i = 1 : n_corners
        corners_stats{i}.speed = speed(idx_corners(i));
        corners_stats{i}.brake_speed = speed(idx_braking(i));
        corners_stats{i}.time_rise_accel = t_rise_accelerate(i);
        corners_stats{i}.time_rise_brake = -t_rise_brake(i);
    end
end