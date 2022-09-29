clear all

full_gps_mode = 0;

% Import
%D:\Seafile\These Pierre\Code\Matlab\Scripts_analyse\data_tracking\function_1\plot
load('data_test_full_gps.mat')
load('data_test_no_gps_3.mat')
addpath(genpath('...\Document\article_4'))
addpath(genpath('...\\Autre travaux\tagtools_release_28-Feb-2018'))

% Store data

%Each line equal 1 dive sequence
first_s = 1;
last_s = 4;
inc = 1;
nb_pos = 21;

%Var
first_ned_x = 0;
first_ned_y = 0;
first_ned_z = 0;
first_eth_x = 0;
first_eth_y = 0;
first_eth_z = 0;

for i = first_s:last_s


    
len = length(sequence_string(1,:)); % Length of receive data

%Traj

%Search idx of first position
if full_gps_mode == 1
    
    for j=1:len
        if strcmp('gps_lat_01',sequence_string(1,j))
            idx_first_lat = j;
            idx_last_lat = j + nb_pos -1;
        end
        if strcmp('gps_lng_01',sequence_string(1,j))
            idx_first_lon = j;
            idx_last_lon = j + nb_pos -1 ;
        end
    end

dive(inc,1).var.LAT.data = sequence(i,idx_first_lat:idx_last_lat);
dive(inc,1).var.LON.data = sequence(i,idx_first_lon:idx_last_lon);



end

if full_gps_mode == 0
    
    for j=1:len
        
        
        %Duration
        if strcmp('lum_sensor_01',sequence_string(1,j))
        dive(inc,1).duration = sequence(i,j);
        end
        
        %GPS LAT ANCHOR
        if strcmp('gps_lat_01',sequence_string(1,j))
            idx_first_lat = j;
            
        end
        
        % GPS LON ANCHOR
        if strcmp('gps_lng_01',sequence_string(1,j))
            idx_first_lon = j;
           
        end
        
        %X NED
        if contains(sequence_string(1,j),'ned_x') && first_ned_x == 0
            first_ned_x = 1;
            idx_first_ned_x = j;  
            idx_last_ned_x = j + nb_pos -1 ;
        end
        
        %Y NED
        if contains(sequence_string(1,j),'ned_y') && first_ned_y == 0
            first_ned_y = 1;
            idx_first_ned_y = j;    
            idx_last_ned_y = j + nb_pos -1 ;
        end
        
        %Z NED
        if contains(sequence_string(1,j),'ned_z') && first_ned_z == 0
            first_ned_z = 1;
            idx_first_ned_z = j;   
            idx_last_ned_z = j + nb_pos -1 ;
        end
        
        %1 : Start behavior 2nd
        if contains(sequence_string(1,j),'acc_x') && first_eth_x == 0
           first_eth_x = 1;
           idx_first_eth_x = j; 
           
        end
        
        %1 : Last behavior 1rst 
         if contains(sequence_string(1,j),'acc_x') && ~contains(sequence_string(1,j+1),'acc_x') && first_eth_x == 1
            idx_last_eth_x = j;
         end
        
        % 2 : Start behavior 2nd
        if contains(sequence_string(1,j),'acc_y') && first_eth_y == 0
           first_eth_y = 1;
           idx_first_eth_y = j; 
           
        end
        
        % 2 : Last behavior 2nd 
         if contains(sequence_string(1,j),'acc_y') && ~contains(sequence_string(1,j+1),'acc_y') && first_eth_y == 1
            idx_last_eth_y = j;
         end
         
        % 3 : Start behavior 3nd
        if contains(sequence_string(1,j),'acc_z') && first_eth_z == 0
            first_eth_z = 1;
           idx_first_eth_z = j; 
           
        end
        % 3 :Last behavior 3nd 
         if contains(sequence_string(1,j),'acc_z') && ~contains(sequence_string(1,j+1),'acc_z') && first_eth_z == 1
            idx_last_eth_z = j;
         end
            
            
    end

    if inc==1
       dive(inc,1).start =  1; 
       dive(inc,1).tend  =  dive(inc,1).duration;  
    else
      dive(inc,1).start =  dive(inc-1,1).tend+1; 
      dive(inc,1).tend  = dive(inc,1).start + dive(inc,1).duration; 
    end
        


lat0 = sequence(i,idx_first_lat);
lon0 = sequence(i,idx_first_lon);

dive(inc,1).var.X.data(:,1) = sequence(i,idx_first_ned_x:idx_last_ned_x)*10; % X NED
dive(inc,1).var.Y.data(:,1) = sequence(i,idx_first_ned_y:idx_last_ned_y)*10; % Y NED
dive(inc,1).var.P.data(:,1) = sequence(i,idx_first_ned_z:idx_last_ned_z)*10; % Z NED / DEPTH

x = dive(inc,1).var.X.data;
y = dive(inc,1).var.Y.data;
inc_eth =1;
for jj = idx_first_eth_y:idx_last_eth_y
    if ~isnan(sequence(i,jj))
    dive(inc,1).var.B.data(inc_eth,1) = sequence(i,jj); % Behavior
    inc_eth = inc_eth+1;
    end
end
inc_eth =1;
for jj = idx_first_eth_z:idx_last_eth_z
    if ~isnan(sequence(i,jj))
    dive(inc,1).var.B.data(inc_eth,4) = sequence(i,jj)*100; % Behavior
    inc_eth = inc_eth+1;
    end
end
inc_eth =1;
for jj = idx_first_eth_x:idx_last_eth_x
    if ~isnan(sequence(i,jj))
    dive(inc,1).var.B.data(inc_eth,2) = sequence(i,jj)*100; % Behavior
    inc_eth = inc_eth+1;
    end
end


for jj = 1:length(dive(inc,1).var.B.data(:,2))-1

        dive(inc,1).var.B.data(jj,3) =  dive(inc,1).var.B.data(jj+1,2)-1; % end behavior

end

dive(inc,1).var.B.data(end,3) = dive(inc,1).duration; % end behavior

%Timestamp (only if NP algorithm)
dive(inc,1).var.T.data(1,1) = 0;
dt = floor(dive(inc,1).duration/(nb_pos-1));
len_d = length(dive(inc,1).var.X.data(:,1));
for ii=2:len_d-1
    dive(inc,1).var.T.data(ii,1) = dt*(ii-1);
end
dive(inc,1).var.T.data(len_d,1) = dive(inc,1).duration;


%Convert to lat, lon
wgs84 = wgs84Ellipsoid;
[dive(inc,1).var.LAT.data(:,1),dive(inc,1).var.LON.data(:,1)] = ned2geodetic(x,y,0,lat0,lon0,0,wgs84); % LAT & LON & ALT

%% Resample
dive_i(inc,1) = dive(inc,1);

lat = dive(inc,1).var.LAT.data(:,1);
lon = dive(inc,1).var.LON.data(:,1);
tx =  dive(inc,1).var.T.data(:,1);
ty= (1:dive(inc,1).duration)';

dive_i(inc,1).var.LAT.data = interp1(tx,lat,ty);
dive_i(inc,1).var.LON.data = interp1(tx,lon,ty);

dive_i(inc,1).var.LAT.data = fillmissing(dive_i(inc,1).var.LAT.data, 'next');
dive_i(inc,1).var.LON.data = fillmissing(dive_i(inc,1).var.LON.data, 'next');

end

inc = inc+1;

end


%% Correction

t=0;
%Find anchor positions

for i= 1:length(dive_i(:,1))-1   
    

    t = dive_i(i,1).duration;
    T_COR(i,1) = t-1;
    POS_COR(i,1) = dive_i(i+1,1).var.LAT.data(1,1);
    POS_COR(i,2) = dive_i(i+1,1).var.LON.data(1,1);
    % TODO solve problem when first is NaN
    POS_COR(2,1) = -21.1725832369987;
    POS_COR(2,2) = 55.286285620470060;
    
D = ([dive_i(i,1).var.LAT.data(:,1), dive_i(i,1).var.LON.data(:,1)]);
[D1,C] = fit_tracks(POS_COR(i,:),T_COR(i,:),D,1);

dive_i(i,1).var.LAT.data(:,1)= D1(:,1);
dive_i(i,1).var.LON.data(:,1)= D1(:,2);
%[D,C] = fit_track_behav(P,T,D,seq,1);

% plot(D(:,1),D(:,2),'x');
% hold on
% plot(D1(:,1),D1(:,2),'x');

end

%% Plot

nb_dive = length(dive(:,1));

GPS = gpxread('2022-09-06_6_sept._2022_19_35_36'); 
first_gpx=1;
last_gpx=75;
gps.lat = GPS.Latitude';
gps.lon = GPS.Longitude';


figure(11)
t = tiledlayout(4,5,'Padding','compact');%,'TileSpacing' ,'compact');

geop = geoaxes(t);
geop.Layout.Tile = 1;
geop.Layout.TileSpan = [4 2];

%Merge dives to a global sequence
seq = merge_dives2seq_TEST(dive_i(:,1));
bev = seq(1,1).var.B.data(:,:);

geoplot(seq(1,1).var.LAT.data,seq(1,1).var.LON.data,'LineWidth',2,...
    'Color',[0.67 0.61 0.92])
hold on

%legend1 = legend('Trajectory corrected','Reference','AutoUpdate','off');


for i = 1:nb_dive
geoplot(dive(i,1).var.LAT.data,dive(i,1).var.LON.data,'*','MarkerSize',10,...
    'MarkerEdgeColor',[0.67 0.61 0.92]);


hold on;
%geoplot(dive_i(i,1).var.LAT.data,dive_i(i,1).var.LON.data,'r')
geobasemap("satellite");
end


inc = 1;
for i =1:length(seq(1,1).var.B.data(:,1))
    
    if seq(1,1).var.B.data(i,1) == 5
        
        t_b(i,1) = seq(1,1).var.B.data(i,3);
        size(i,1) = (seq(1,1).var.B.data(i,3)-seq(1,1).var.B.data(i,2))*1.5;
        POS_SURF(inc,:) = [seq(1,1).var.LAT.data(t_b(i,1),1), seq(1,1).var.LON.data(t_b(i,1),1)];
        geoplot(POS_SURF(inc,1),  POS_SURF(inc,2), '.', 'MarkerSize',size(i,1), 'Color',[0.98 1 0.67])
        inc = inc +1;
    end
    
end

inc = 1;
for i =1:length(seq(1,1).var.B.data(:,1))
    
    if seq(1,1).var.B.data(i,1) == 2
        
        t_b(i,1) = seq(1,1).var.B.data(i,3);
        size(i,1) = (seq(1,1).var.B.data(i,3)-seq(1,1).var.B.data(i,2))*1.5;
        POS_REST(inc,:) = [seq(1,1).var.LAT.data(t_b(i,1),1), seq(1,1).var.LON.data(t_b(i,1),1)];
        geoplot(POS_REST(inc,1),POS_REST(inc,2), '.', 'MarkerSize',size(i,1), 'Color',[0.71 0.71 0.71])
        inc = inc +1;
    end
    
end

%Plot uncertainty circle GPS
geoplot(dive_i(1,1).var.LAT.data(1,1), dive_i(1,1).var.LON.data(1,1), 'o','LineWidth',2, 'MarkerSize',20, 'Color',[0 0 0])

for i =1:length(dive_i(:,1))
    
        size(i,1) = 20;
        geoplot(dive_i(i,1).var.LAT.data(end,1), dive_i(i,1).var.LON.data(end,1), 'o','LineWidth',2, 'MarkerSize',size(i,1), 'Color',[0 0 0])
        
end

%Plot uncertainty circle position
% for i =1:length(seq(:,1))
%     
%         size(i,1) = 15;
%         geoplot(dive_i(1,1).var.LAT.data(1,1), dive_i(1,1).var.LON.data(1,1), 'o', 'MarkerSize',size(i,1), 'Color',[0 0 0])
%         
% end

% geoplot(D1(:,1),D1(:,2),'r*')
geoplot(gps.lat(first_gpx:last_gpx,1),gps.lon(first_gpx:last_gpx,1),'g*')
geobasemap("satellite");
%geoplot(gps.lat(first_gpx:last_gpx,1),gps.lon(first_gpx:last_gpx,1),'g')

geop.LatitudeAxis.TickLabelRotation = 90;
geop.Basemap = 'satellite';
hold off
box on
% axis tight
% axis equal
% ylabel('East-West (m)')
% xlabel('North-South (m)')

 etho_color(1,1:3) =  [0.87 0.79 0.99];
 etho_color(3,1:3) =  [0.98 1 0.67];
 etho_color(2,1:3) =  [0.71 0.71 0.71];
 %etho_color(7,1:3) =  [1 1 1];

 tickLbls(3,1) = {'SURFACE'};
 tickLbls(2,1) = {'REST'};
 tickLbls(1,1) =  {'SWIM'};
 %tickLbls(7,1) = {'other'};
 
%clrs = lines(numel(unique(behavior))-1);
clrs = etho_color;
cmap = fliplr(clrs);
%tickLbls = categories(unique(behavior_s{sequence_nb}))';
% tickLbls = tickLbls';
colormap(gca,clrs);
numCategories = numel(unique(bev(:,1)));
Ticks = 1/(numCategories*2):1/numCategories:1;
c = colorbar;
c.Location = 'east';
c.TickLabels = tickLbls;
c.Ticks = Ticks;
c.TickLength = 0;



nexttile([2 3])
plot_post_pro(seq.var.LAT.data,seq.var.B.data,1,'Latitude')
nexttile([2 3])
plot_post_pro(seq.var.LON.data,seq.var.B.data,1,'Longitude')




