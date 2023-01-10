clear all
clc
close all
set(groot,'defaultAxesFontName','Courier');
global background_color text_color

background_color =  [13/255, 17/255, 23/255];
text_color = [201,209,217]/255;

% (1) Script inputs
study_cases = {dir('data/f1_*.xml').name};

accumulated_di = 0;
for study_case_ = study_cases
    study_case = study_case_{:};
    
    data_ref = read_optimal_laptime(['old-references/',study_case]);
    data_new = read_optimal_laptime(['data/',study_case]);
    
    if ~isfield(data_new,'time')
        continue;
    end

    h = figure('Color',background_color);
    plot_timeseries(1,data_new.road_arclength, data_ref.chassis_velocity_x, data_new.chassis_velocity_x, 'Velocity-x [m/s]');
    legend({'reference','new'},'TextColor',text_color);
    plot_timeseries(2,data_new.road_arclength, data_ref.chassis_velocity_y, data_new.chassis_velocity_y, 'Velocity-y [m/s]');
    plot_timeseries(3,data_new.road_arclength, data_ref.chassis_omega_z, data_new.chassis_omega_z, 'Omega-y [rad/s]');
    plot_timeseries(4,data_new.road_arclength, data_ref.control_variables.chassis_throttle*100, data_new.control_variables.chassis_throttle*100, 'Throttle [%]');
    plot_timeseries(5,data_new.road_arclength, data_ref.control_variables.front_axle_steering_angle, data_new.control_variables.front_axle_steering_angle, 'Steering angle [rad]');
    plot_timeseries(6,data_new.road_arclength, data_ref.road_track_heading_angle, data_new.road_track_heading_angle, 'Track heading angle [rad]');
    try
    plot_timeseries(7,data_new.road_arclength, data_ref.chassis_Fz_fl, data_new.chassis_Fz_fl, 'Fz [N]');
    plot_timeseries(7,data_new.road_arclength, data_ref.chassis_Fz_fr, data_new.chassis_Fz_fr, 'Fz [N]');
    plot_timeseries(7,data_new.road_arclength, data_ref.chassis_Fz_rl, data_new.chassis_Fz_rl, 'Fz [N]');
    plot_timeseries(7,data_new.road_arclength, data_ref.chassis_Fz_rr, data_new.chassis_Fz_rr, 'Fz [N]'); box on; grid minor;
    end
    plot_timeseries(8,data_new.road_arclength, data_ref.front_axle_left_tire_kappa, data_new.front_axle_left_tire_kappa, 'kappa [-]');
    plot_timeseries(8,data_new.road_arclength, data_ref.front_axle_right_tire_kappa, data_new.front_axle_right_tire_kappa, 'kappa [-]');
    plot_timeseries(8,data_new.road_arclength, data_ref.rear_axle_left_tire_kappa, data_new.rear_axle_left_tire_kappa, 'kappa [-]');
    plot_timeseries(8,data_new.road_arclength, data_ref.rear_axle_right_tire_kappa, data_new.rear_axle_right_tire_kappa, 'kappa [-]'); box on; grid minor;
    
    try
        sgtitle([strrep(study_case,'_','\_'),' dt: ',num2str(data_new.laptime-data_ref.laptime), ' di: ',num2str(data_new.optimization_data.number_of_iterations-data_ref.optimization_data.number_of_iterations)],'Color',text_color)
    catch
        sgtitle([strrep(study_case,'_','\_'),' dt: ',num2str(data_new.laptime-data_ref.laptime)],'Color',text_color)
    end
    h.Position = [680    42   997   954];
    set(h, 'InvertHardcopy', 'off');
    print(['old-references/',strrep(study_case,'.xml','.png')], '-dpng','-r300')
    close(h);

    try
    accumulated_di = accumulated_di + data_new.optimization_data.number_of_iterations-data_ref.optimization_data.number_of_iterations;
    end
end

%mergePdfs('new-references/',study_cases, 'new-references-study.pdf')

function plot_timeseries(i_frame, s, data_ref, data_new, title_str)
global background_color text_color


blue   = [0   0.447000000000000   0.741];
orange = [0.850000000000000   0.325000000000000   0.098000000000000];

color_ref = orange;
color_new = blue;
subplot(8,1,i_frame); hold on; box on; grid minor;
h = gcf;
h.CurrentAxes.Color = background_color;
h.CurrentAxes.XAxis.Color = text_color;
h.CurrentAxes.YAxis.Color = text_color;
plot(s, data_ref,'Color',color_ref);
plot(s, data_new,'Color',color_new);
xlim([0,s(end)])
title(title_str,'Color',text_color)
set(gca,'MinorGridLineStyle','-')
end

function data = read_optimal_laptime(study_case)
data = readstruct(study_case);

fields = fieldnames(data);
for field = 1:numel(fields)
    if ( ~isnumeric( data.(fields{field})) && ~isstruct(data.(fields{field})) )
        data.(fields{field})= cell2mat(textscan(data.(fields{field}),'%f', 'delimiter',','));
    elseif ( strcmpi(fields{field}, 'control_variables') )
        subfields = fieldnames(data.(fields{field}));
        for subfield = 1:numel(subfields)
            if ( ~isnumeric( data.(fields{field}).(subfields{subfield}).values ) )
                data.(fields{field}).(subfields{subfield}) = cell2mat(textscan(data.(fields{field}).(subfields{subfield}).values,'%f', 'delimiter',','));
            end
        end
        
    end
end
end

function mergePdfs(path,fileNames, outputFile)

memSet = org.apache.pdfbox.io.MemoryUsageSetting.setupMainMemoryOnly();
merger = org.apache.pdfbox.multipdf.PDFMergerUtility;
fileNames = cellfun(@(f)([path,strrep(f,'.xml','.pdf')]),fileNames,'UniformOutput',false);
cellfun(@(f) merger.addSource(f), fileNames)

merger.setDestinationFileName(outputFile)
merger.mergeDocuments(memSet)
end
