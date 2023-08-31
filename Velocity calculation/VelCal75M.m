% Author- Deependra Kumar 
% Date Modified- 4 July 2023
% Objective- To find Impact velocity of droplet
clear all; close all;

%% Inserting video properties
vid = VideoReader('75GSM.mp4');
videoFileReader = vision.VideoFileReader('75GSM.mp4');
S = info(videoFileReader);
frameRate = 2000; % frame/second (Actual one)
width = vid.Width; height = vid.Height; % pixel
%% calculation of pixel parameter to mm
dia_mm = 2.65; % droplet dia for calibration (mm)
dia_ref = 83; % dia of 5th frame for calibration (pixel)- 
% calculated dia in pixel first the extracted 5th frame dia
width_mm = (width*dia_mm)/dia_ref; % width of frames in mm
height_mm = (height*dia_mm)/dia_ref; % height of frames in mm
scale = height_mm/height; %mm/pixel
num_frames = vid.NumFrames;

%% Centroid Calculation of droplet
j = 1;
while ~isDone(videoFileReader)
    videoFrame = step(videoFileReader);
   
    hsv = rgb2hsv(videoFrame);
    hImage = hsv(:,:,1);
    sImage = hsv(:,:,2); % we will be using this (saturation) one
    vImage = hsv(:,:,3);
    bw = im2bw(sImage); %Binary Image
%     figure(), imshow(bw);
    stats = [regionprops(bw)];
    Area = extractfield(stats,'Area'); % extracting Area from stats for sorting 
    sort_area = sort(Area,'descend');
    t = struct2table(stats);
%     points = t.Centroid;
    for i = 1:numel(stats) 
        if Area(i)>sort_area(2) 
%             rectangle('Position', stats(i).BoundingBox, ...
%         'Linewidth', 1, 'EdgeColor', 'r');
            t = struct2table(stats(i));
            points(j,:) = t.Centroid;     
        end
    end
j = j+1;

end

%% Velocity calculation (frame-by-frame)
Velocity = [];  %points(num_frames+1,:) = [0 0];
k = 1;
for i = 27:num_frames-1
    frames = read(vid,i);
    vel_pix = abs(points(i+1,:) - points(i,:)); %velocity (pixels/frame)
    vel = vel_pix(2) * frameRate*scale; % pixels/frame * frame/seconds  = pixel/sec
    Velocity(k) = vpa(vel/1000,4); % Velocity in m/s

    
    k = k+1;
end
Time = 1000*(num_frames/frameRate) % Time in mili second
T = linspace(0,Time,115); %Impact Velocity time from 27 frame to end = 0.013 to 0.0705 divided into 115
plot(T,Velocity,'LineWidth',1,'Marker','.','MarkerEdgeColor','r');
xlabel('Time (msec)'); ylabel('Velocity (m/sec)');
title('Droplet Impact Velocity - time plot of 75GSM Video'); 
ax = gca; %axis('tight');
ax.TitleHorizontalAlignment = 'left';
