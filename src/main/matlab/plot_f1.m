function h_im = plot_f1(x_rr,y_rr,psi,wb,track,varargin)

if ( nargin > 5 )
    skin = varargin{1};
else
    skin = 'generic';
end

if ( nargin > 6 )
    color = varargin{2};
else
    color = [1.0,1.0,1.0];
end

if ( nargin > 7 )
    FaceAlpha = varargin{3};
else
    FaceAlpha = 1.0;
end

if (nargin > 8 )
    scale_factor = varargin{4};
else
    scale_factor = 1;
end
    

if ( strcmpi(skin,'generic') )
    [im_original,~,alpha_original] = imread('f1_marker.png');
    
    tire_rr = [659,2930];
    tire_rl = [1518,2930];
    tire_fr = [659,1055];
    scale = 4;
elseif ( strcmpi(skin, 'redbull') )
    [im_original,~,alpha_original] = imread('redbull2022_marker.png');
    
    tire_rr = [135, 739];
    tire_rl = [358, 739];
    tire_fr = [135, 257];
    scale = 1;
    
elseif ( strcmpi(skin, 'ferrari') )
    [im_original,~,alpha_original] = imread('ferrari2022.png');
    alpha_original(1:400,:) = 0.0;
    tire_rr = [542, 2088];
    tire_rl = [1162, 2088];
    tire_fr = [542, 680];
    scale = 2;

elseif strcmpi(skin, 'mercedes')
    [im_original,~,alpha_original] = imread('mercedes2022.png');
    im_original = imrotate(im_original, 180);
    alpha_original = imrotate(alpha_original, 180);
    
    tire_rr = [56, 856];
    tire_rl = [323, 856];
    tire_fr = [56, 216];
    
    scale = 2*scale_factor;

elseif strcmpi(skin, 'alpine')
    [im_original,~,alpha_original] = imread('alpine2022_marker.png');
    
    tire_rr = [173, 1563];
    tire_rl = [650, 1563];
    tire_fr = [173, 444];
    
    scale = 2*scale_factor;

else
    error('Skin name not recognized');
end

alpha_original(alpha_original < 250) = 0;

im = imresize(im_original,[size(im_original,1)/scale,size(im_original,2)/scale]);
alpha = imresize(alpha_original,[size(im_original,1)/scale,size(im_original,2)/scale]);



%im_light = imresize(im_original,[size(im_original,1)/(scale*2),size(im_original,2)/(scale*2)]);

% put im_light to zeros;
im_light = zeros(round(size(im_original,1)/(scale*2)),round(size(im_original,2)/(scale*2)),3);

for i = -1:1
    for j = -1:1
        im_light(round(tire_rr(1)/(scale*2))+i,round(tire_rr(2)/(scale*2))+j,:) = [255,0,0];
        im_light(round(tire_rl(1)/(scale*2))+i,round(tire_rl(2)/(scale*2))+j,:) = [0,255,0];
        im_light(round(tire_fr(1)/(scale*2))+i,round(tire_fr(2)/(scale*2))+j,:) = [0,0,255];
    end
end

%
%   (1) Scale the image to achieve the ratio between wb and track
%       Keep the wheelbase, squish the track
track_img = norm(tire_rr-tire_rl);
wb_img = norm(tire_fr-tire_rr);
track_to_wb_desired = track/wb;
track_to_wb_img = track_img/wb_img;
im_scaled = imresize(im_light,[size(im_light,1)*track_to_wb_desired/track_to_wb_img,size(im_light,2)]);

%
%   (2) Perform the rotation
im_rotated = imrotate(im_scaled,-psi,'bilinear');

%
%   (3) Find the coordinates of the rear right tire (colored in red)
found = false;
for i = 1:size(im_rotated,1)
    for j = 1:size(im_rotated,2)
        pix = im_rotated(i,j,:);
        pix = pix(:);
        if ( norm(double(pix) - [255;0;0]) < 30 )
            tire_rr = [i,j];
            found = true;
            break;
        end
    end
    if ( found )
        break;
    end
end

%
% (4) Find the coordinates of the rear left tire (colored in green)
found = false;
for i = 1:size(im_rotated,1)
    for j = 1:size(im_rotated,2)
        pix = im_rotated(i,j,:);
        pix = pix(:);
        if ( norm(double(pix) - [0;255;0]) < 40 )
            tire_rl = [i,j];
            found = true;
            break;
        end
    end
    if ( found )
        break;
    end
end

%
% (5) Find the coordinates of the front right tire (colored in blue)
found = false;
for i = 1:size(im_rotated,1)
    for j = 1:size(im_rotated,2)
        pix = im_rotated(i,j,:);
        pix = pix(:);
        if ( norm(double(pix) - [0;0;255]) < 70 )
            tire_fr = [i,j];
            found = true;
            break;
        end
    end
    if ( found )
        break;
    end
end

%
%   (6) Compute the physical coordinates solving a least squares problem
A = [0 0 1-(tire_rr(1)-1)/(size(im_rotated,1)-1) (tire_rr(1)-1)/(size(im_rotated,1)-1); ...
    1-(tire_rr(2)-1)/(size(im_rotated,2)-1) (tire_rr(2)-1)/(size(im_rotated,2)-1) 0 0; ...
    0 0 1-(tire_rl(1)-1)/(size(im_rotated,1)-1) (tire_rl(1)-1)/(size(im_rotated,1)-1); ...
    1-(tire_rl(2)-1)/(size(im_rotated,2)-1) (tire_rl(2)-1)/(size(im_rotated,2)-1) 0 0; ...
    0 0 1-(tire_fr(1)-1)/(size(im_rotated,1)-1) (tire_fr(1)-1)/(size(im_rotated,1)-1); ...
    1-(tire_fr(2)-1)/(size(im_rotated,2)-1) (tire_fr(2)-1)/(size(im_rotated,2)-1) 0 0];

b = [y_rr;x_rr; y_rr - track*cosd(psi); x_rr - track*sind(psi);y_rr + wb*sind(psi); x_rr - wb*cosd(psi)];
x = A\b;
x_range = x(1:2);
y_range = x(3:4);

% Repeat all the operations on the high quality photo, and represent it
im_scaled = imresize(im,[size(im,1)*track_to_wb_desired/track_to_wb_img,size(im,2)]);
alpha_scaled = imresize(alpha,[size(alpha,1)*track_to_wb_desired/track_to_wb_img,size(alpha,2)]);

im_rotated = imrotate(im_scaled,-psi,'bilinear');
alpha_rotated = imrotate(alpha_scaled,-psi,'bilinear');

for i = 1 : size(im_rotated,1)
    for j = 1 : size(im_rotated,2)
        for k = 1 : 3
            im_rotated(i,j,k) = double(im_rotated(i,j,k))*color(k);
        end
    end
end

h_im = imagesc(x_range,y_range,im_rotated);
h_im.AlphaData = FaceAlpha*alpha_rotated;


end
