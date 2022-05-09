clear all
%clc
%close all
format long

% (1) Load library
if libisloaded('libfastestlapc')
    unloadlibrary libfastestlapc
end

loadlibrary('../../../build/lib/libfastestlapc.dylib','../../../src/main/c/fastestlapc.h');

% (2) Construct track
options = '<options> <save_variables> <prefix>track/</prefix> <variables> <s/> </variables> </save_variables> </options>';
circuit = calllib('libfastestlapc','create_track',[],'melbourne','../../../database/tracks/melbourne/melbourne_adapted.xml',options);


len_s = calllib("libfastestlapc","download_vector_table_variable_size",'track/s');
s = zeros(1,len_s);
s = calllib("libfastestlapc","download_vector_table_variable",s,length(s), 'track/s');

s_lec = [33.83, 47.51888888888889, 64.68555555555555, 88.71055555555557, 112.97722222222224, 140.79944444444445, 154.75500000000002, 179.1772222222222, 196.62166666666667, 210.57722222222225, 234.99944444444446, 248.8216666666667, 265.8216666666667, 297.15500000000003, 317.15500000000003, 327.7661111111111, 338.0438888888889, 350.31055555555554, 362.5772222222222, 381.5772222222222, 394.63138888888886, 403.6536111111111, 417.65361111111105, 441.8758333333333, 459.2980555555555, 485.0758333333332, 498.13138888888875, 516.7980555555555, 535.7855555555554, 558.0077777777777, 569.2522222222221, 586.4522222222222, 598.0522222222222, 618.5855555555555, 648.6966666666666, 679.3633333333333, 694.9188888888889, 707.4522222222222, 723.1744444444444, 752.0744444444445, 778.1188888888889, 801.0633333333333, 830.7633333333332, 860.8466666666666, 874.2244444444443, 897.8688888888887, 918.2022222222221, 935.0911111111109, 962.0244444444443, 982.9466666666666, 998.5466666666666, 1007.7956666666666, 1020.5466666666665, 1031.2133333333331, 1037.6616666666664, 1045.1061111111107, 1055.9505555555552, 1062.4505555555552, 1068.728333333333, 1079.628333333333, 1088.0283333333332, 1095.3616666666665, 1103.0283333333332, 1112.5949999999998, 1126.0949999999998, 1142.9838888888887, 1151.9838888888887, 1161.4838888888887, 1174.8616666666665, 1185.9283333333333, 1200.0166666666664, 1212.461111111111, 1223.1944444444443, 1230.6166666666666, 1240.172222222222, 1252.1055555555554, 1262.4388888888886, 1270.972222222222, 1286.6833333333332, 1300.5499999999997, 1322.09, 1336.8899999999999, 1362.7788888888886, 1375.9455555555553, 1405.7677777777774, 1430.5677777777776, 1450.012222222222, 1475.4122222222218, 1501.112222222222, 1524.134444444444, 1553.4677777777774, 1574.3899999999996, 1598.7324999999994, 1626.3324999999993, 1644.8658333333328, 1657.3102777777772, 1682.4658333333327, 1714.2436111111103, 1730.1880555555547, 1742.9436111111102, 1758.8880555555545, 1780.432499999999, 1794.5991666666655, 1813.0441666666654, 1825.7108333333322, 1842.9774999999986, 1855.1441666666651, 1867.3663888888875, 1894.9886111111095, 1907.8219444444428, 1918.221944444443, 1931.4441666666653, 1944.7219444444431, 1966.499722222221, 1994.2774999999986, 2008.3886111111096, 2019.8108333333319, 2040.1886111111098, 2069.929444444443, 2084.929444444443, 2099.9849999999988, 2115.1516666666653, 2145.8183333333322, 2167.362777777776, 2198.5849999999987, 2226.884999999999, 2249.051666666666, 2274.5627777777763, 2293.6827777777767, 2316.2383333333323, 2329.1716666666653, 2365.104999999999, 2381.438333333332, 2401.1716666666657, 2434.2827777777766, 2457.538333333332, 2474.2049999999986, 2500.960555555554, 2517.6827777777767, 2534.4605555555545, 2564.5605555555544, 2584.827222222221, 2608.634166666666, 2629.0341666666654, 2652.7563888888876, 2666.4008333333322, 2697.100833333332, 2710.745277777776, 2734.5452777777764, 2754.9452777777765, 2771.945277777776, 2792.345277777776, 2809.400833333332, 2839.915833333331, 2856.9158333333316, 2873.915833333332, 2907.915833333332, 2921.515833333332, 2941.8491666666655, 2968.871388888888, 2995.982499999999, 3009.538055555555, 3023.09361111111, 3036.649166666665, 3056.9824999999987, 3080.6269444444433, 3094.182499999999, 3121.2047222222204, 3138.0936111111096, 3151.644444444443, 3181.7444444444427, 3195.0777777777757, 3219.788888888887, 3237.455555555554, 3262.6555555555537, 3284.6111111111095, 3309.2111111111094, 3328.6555555555537, 3353.3555555555536, 3364.2444444444427, 3383.2319444444424, 3438.565277777776, 3463.965277777776, 3481.0986111111088, 3504.3874999999975, 3522.0541666666645, 3555.543055555553, 3583.3430555555537, 3598.898611111109, 3614.676388888887, 3630.64472222222, 3643.4002777777755, 3669.2669444444423, 3692.133611111109, 3715.1558333333314, 3738.3336111111093, 3751.6669444444424, 3778.511388888887, 3802.0780555555534, 3815.6780555555533, 3846.378055555553, 3866.9780555555535, 3884.2558333333313, 3901.502499999998, 3929.32472222222, 3943.280277777775, 3957.1913888888866, 3987.0913888888867, 4007.4691666666645, 4020.3580555555536, 4036.535833333331, 4047.3358333333313, 4057.202499999998, 4069.802499999998, 4081.202499999998, 4089.9136111111093, 4095.0119444444426, 4100.123055555554, 4108.123055555554, 4115.17861111111, 4125.989722222221, 4143.545277777776, 4150.789722222221, 4162.5230555555545, 4176.756388888888, 4189.6897222222215, 4207.911944444444, 4219.578611111111, 4231.523055555555, 4256.682222222222, 4274.804444444444, 4287.86, 4313.748888888888, 4339.082222222221, 4354.082222222221, 4378.859999999999, 4388.859999999999, 4403.926666666665, 4414.193333333333, 4429.793333333333, 4440.415555555555, 4461.993888888888, 4489.660555555554, 4500.727222222222, 4516.793888888888, 4532.2716666666665, 4540.227222222222, 4547.649444444444, 4558.616111111111, 4565.727222222223, 4574.5938888888895, 4583.749444444445, 4592.282777777778, 4603.282777777778, 4615.1419444444455, 4626.541944444445, 4633.375277777778, 4639.019722222223, 4653.319722222223, 4660.119722222224, 4671.053055555557, 4689.830833333335, 4701.164166666668, 4710.608611111113, 4721.941944444446, 4733.408611111113, 4743.235277777779, 4766.579722222223, 4775.24638888889, 4784.135277777778, 4802.6241666666665, 4826.846388888889, 4844.579722222223, 4865.5575, 4907.390833333334, 4924.324166666667, 4944.624166666667, 4977.427222222223, 4998.582777777779, 5020.127222222223, 5038.860555555558, 5064.193888888891, 5086.516111111113, 5105.9161111111125, 5141.971666666669, 5175.193888888891, 5212.433888888891, 5225.989444444446];
u_lec = [306, 308, 309, 310, 312, 313, 314, 314, 314, 314, 314, 311, 306, 235, 200, 191, 185, 184, 184, 190, 195, 203, 210, 218, 224, 232, 235, 240, 245, 250, 253, 258, 261, 264, 271, 276, 280, 282, 283, 289, 293, 295, 297, 300, 301, 304, 305, 304, 303, 269, 234, 207, 192, 160, 146, 134, 122, 117, 113, 109, 108, 110, 115, 123, 135, 152, 162, 171, 172, 166, 158, 160, 161, 167, 172, 179, 186, 192, 202, 208, 216, 222, 233, 237, 244, 248, 250, 254, 257, 259, 264, 269, 273, 276, 278, 280, 283, 286, 287, 287, 287, 277, 255, 238, 228, 222, 219, 220, 226, 231, 234, 238, 239, 245, 250, 254, 257, 262, 267, 270, 271, 273, 276, 277, 281, 283, 285, 287, 288, 290, 291, 294, 294, 296, 298, 299, 300, 301, 301, 302, 301, 304, 305, 306, 305, 307, 307, 307, 306, 306, 306, 306, 307, 306, 306, 306, 306, 306, 305, 304, 305, 305, 305, 305, 305, 304, 305, 304, 304, 303, 301, 300, 278, 265, 252, 247, 246, 250, 247, 245, 245, 249, 254, 257, 262, 265, 274, 278, 280, 284, 286, 287, 291, 294, 296, 298, 300, 302, 303, 306, 307, 309, 311, 312, 313, 314, 313, 299, 262, 232, 182, 162, 148, 126, 114, 112, 114, 115, 120, 127, 139, 158, 163, 176, 183, 194, 205, 210, 215, 227, 233, 235, 233, 228, 225, 223, 225, 226, 231, 234, 239, 242, 249, 249, 241, 199, 179, 167, 141, 128, 114, 103, 96, 99, 107, 114, 123, 127, 143, 153, 164, 169, 170, 170, 170, 172, 176, 191, 195, 200, 208, 218, 228, 236, 251, 254, 261, 269, 272, 277, 281, 285, 287, 291, 295, 299, 304, 305];
s_lec = s_lec+36;
s_lec(162:end) = s_lec(162:end)+42;
s_lec = [s_lec(1:198),s_lec(200:end)-s_lec(200)+s_lec(199)];
u_lec = [u_lec(1:198),u_lec(200:end)];

% Make it periodic
ds_lec = s_lec(2)-s_lec(1);
s_lec_pre = s_lec(200:end);
u_lec_pre = u_lec(200:end);
s_lec_full = [s_lec_pre - s_lec_pre(end) + s_lec(1) - ds_lec, s_lec ];
u_lec_full = [u_lec_pre, u_lec];

% Project it into the equally spaced points
u_reference = interp1(s_lec_full,u_lec_full,s);

f = @(x)(fitness_function(circuit,u_reference,s,x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),x(9),x(10),x(11),x(12),x(13),x(14),x(15)));

%x0 = [2.0, 600.0/795.0, 1.0,2.75,0.5,0.35 , 0.0,0.3, 0.5, 2.2,2.2,2.2,2.2,0.55,5000.0/(795.0*9.81)];

%x0=[2.8997422540375357,598.6974062494218742/795,0.9965604049867729,2.9979302913781125,0.5499522228347958,...
%        0.3006017355032958,-0.0525571021443309,0.3258213325509053,0.5598178434393430,2.1827115832115838,...
%        1.9411259238515368,2.1985229259191987,1.5637381248638615,0.5994324686436948,5467.9430728319703121/(795.0*9.81)];

x0=[2.9215533912538554,668.8400387253313966/795.0,1.1256781620536049,2.9846711290170758,0.5476972096461912,...
    0.3011810022825973,-0.0807904677747522,0.3282011514039212,0.4976477031894002,2.1844344498933728,1.7803022190099620,...
    2.1966720738989918,1.4012636913583039,0.6360028480923983,4701.3636904900376976/(795.0*9.81)];


lb = [1.0  ,0.7,0.995,2.5,0.45,0.30,-0.2,0.20,0.20 ,1.4,1.4,1.4,1.4,0.50,4000.0/(795.0*9.81)];
ub = [3.0  ,0.85,1.3,3.0,0.55,0.40, 0.2,0.35,0.80,2.2,2.2,2.2,2.2,0.65,6000.0/(795.0*9.81)];



options = optimoptions('fmincon','TolX',1.0e-4,'TolFun',1.0e-4,'Display','iter','SpecifyObjectiveGradient',true);

x=fmincon(f,x0,[],[],[],[],lb,ub,[],options);


function [f,df] = fitness_function(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2,brake_bias,max_torque)
print_variables(differential_stiffness,power*795.0,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2,brake_bias,max_torque*795.0*9.81);

N = 15;



% Compute the fitness function
f = run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
    mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, false);

fprintf("Fitness function computed: ");
if nargout > 1
    % Compute the gradient
    df = zeros(1,N);
    

        dx = 1.0e-5*10^(differential_stiffness);
        df(1) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness+dx,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
            mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
        fprintf("1 ");

    dx = 1.0e-3;
    df(2) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power+dx,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
    fprintf("2 ");
    dx = 1.0e-3;
    df(3) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd+dx,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
    fprintf("3 ");
    dx = 1.0e-3;
    df(4) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl+dx,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
    fprintf("4 ");
    dx = 1.0e-4;
    df(5) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog+dx,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
    fprintf("5 ");
    dx = 1.0e-4;
    df(6) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog+dx,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
    fprintf("6 ");
    dx = 1.0e-3;
    df(7) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press+dx,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
    fprintf("7 ");
    dx = 1.0e-3;
    df(8) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press+dx, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
    fprintf("8 ");
    dx = 1.0e-3;
    df(9) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance+dx, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
    fprintf("9 ");
    
    
    dx = 1.0e-3;
    df(10) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1+dx, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
    fprintf("10 ");
    dx = 1.0e-3;
    df(11) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2+dx, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
    fprintf("11 ");
    dx = 1.0e-3;
    df(12) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1+dx, mu_y_rear_2, brake_bias, max_torque, true)-f)/dx;
    fprintf("12 ");
    dx = 1.0e-3;
    df(13) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2+dx, brake_bias, max_torque, true)-f)/dx;
    fprintf("13 ");
    dx = 1.0e-3;
    df(14) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias+dx, max_torque, true)-f)/dx;
    fprintf("14 ");
    dx = 1.0e-4;
    df(15) = (run_fastest_lap(circuit,u_reference,s,differential_stiffness,power,cd,cl,x_cog,h_cog,x_press,z_press, roll_balance, ...
        mu_y_front_1, mu_y_front_2, mu_y_rear_1, mu_y_rear_2, brake_bias, max_torque+dx, true)-f)/dx;
    fprintf("15");
end
fprintf("\n");
end

function print_variables(varargin)
for i = 1:nargin
    fprintf("    scalar %s = %24.16f;\n",inputname(i),varargin{i})
end
end
