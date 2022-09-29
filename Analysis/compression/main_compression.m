clear all
close all

% Import tag data in timetables
main_import_tag_data

%% format data tag
[var] = f_tt2var(data_tag,100);
[seq_buf] = f_var2seq(var);

clearvars -except seq_s_ref seq_ref var seq_buf

%% Proccessing ALGO

% downsample 1Hz
seq = downs_seq(seq_buf,100);   %Downsample to 1hz
seq.name = 'algo2';

% Low_pass filter
[seq.var.Ag] = mov_mean_filter(2,seq.var.A);  % For 1hz sampling
[seq.var.Mg] = mov_mean_filter(2,seq.var.M);  % For 1hz sampling

% Compute variables
[seq.var.Ad] = var_accd(seq.var);
[seq.var.O]  = var_odba(seq.var);
[seq.var.J]  = var_jerk(seq.var);

[seq.var.Pi,seq.var.Ro,seq.var.He] = var_saam(seq.var);

% Correction pitch offset
seq.var.Pi.data = seq.var.Pi.data -(9/180*pi);

% Cut seq in dives delimited by surface
surf = find_dives(seq.var.P,0.2);
dives = f_seq2dives(surf,seq);

%Ethogram apply on dive
dives = var_etho_V2(dives);

% Speed estimation in function of the ethogram
v = 2; %Version of the ethogram
speed_method = 1;
[dives] = s_etho_dive2s(dives,v,speed_method);

 % Calculate trajectory and tortuosity
 offset_h = -19.32;%-19.32;
 for i=1:length(dives(:,1))
     dives(i,1).type = "seq";
     [dives(i,1)] = seq_seq2htrack(dives(i,1),offset_h);
     P_tab{i,1}(:,1) = dives(i,1).var.X.data(3:end);
     P_tab{i,1}(:,2) = dives(i,1).var.Y.data(3:end);
     P_tab{i,1}(:,3) = dives(i,1).var.P.data(3:end);
     if length(P_tab{i,1}(:,1)) > 1
     t{i,1}(1,:)= tortuosity(P_tab{i,1}(:,1:2),1,1);
     end
 end
       

%% Sorting Dive : Calculate length dive, Depth max and if resting phase

inc_us = 1;
inc_s = 1;
inc_m = 1;
inc_l = 1;
inc_r = 1;

for i=1:length(dives(:,1))
    
    dive_length(i,1) = dives(i,1).tend-dives(i,1).tstart;
    dive_depth_max(i,1) = max(dives(i,1).var.P.data);
    
    rest_dive = 0;
    % plot(dive_length);
    for j=1:length(dives(i,1).var.B.data(:,1))   % Check if RESTING phase more than 60sec 
        bev = dives(i,1).var.B.data(j,:);
        if  bev(1,1) == 2
            if (bev(1,3) - bev(1,2)) > 60
            rest_dive = 1;
            end
        end
    end
    
    if dive_length(i,1) >= 30 && dive_length(i,1) < 60 && rest_dive == 0  % Check dive length and store 
        
        dive_type(i,1) = 1; 
%         tab_d.us(inc_us,1) = i;
%         tab_d.us(inc_us,2) = dive_length(i,1);
%         tab_d.us(inc_us,3) = dive_depth_max(i,1);
%         inc_us=inc_us+1;
        
    elseif dive_length(i,1) >= 60 && dive_length(i,1) < 150 && rest_dive == 0 % Check dive length and store 
          
        dive_type(i,1) = 2; 
%         tab_d.s(inc_s,1) = i;
%         tab_d.s(inc_s,2) = dive_length(i,1);
%         tab_d.s(inc_s,3) = dive_depth_max(i,1);
%         inc_s =inc_s+1;
        
    elseif dive_length(i,1) >= 150 && dive_length(i,1) < 400 && rest_dive == 0 % Check dive length and store 
        
        
        dive_type(i,1) = 3; 
%         tab_d.m(inc_m,1) = i;
%         tab_d.m(inc_m,2) = dive_length(i,1);
%         tab_d.m(inc_m,3) = dive_depth_max(i,1);
%         inc_m= inc_m+1;

    elseif dive_length(i,1) >= 400 && rest_dive == 0 % Check dive length and store 
        
        dive_type(i,1) = 4;
%         tab_d.l(inc_l,1) = i;
%         tab_d.l(inc_l,2) = dive_length(i,1);
%         tab_d.l(inc_l,3) = dive_depth_max(i,1);
%         inc_l= inc_l+1;    

    elseif rest_dive == 1  % Check dive REST and store 
        
        dive_type(i,1) = 5;
%         tab_d.r(inc_r,1) = i;
%         tab_d.r(inc_r,2) = dive_length(i,1);
%         tab_d.r(inc_r,3) = dive_depth_max(i,1);
%         inc_r= inc_r+1;   
    else
        dive_type(i,1) = 6;
    end
    
end

%% Nb_pts_max and Nb_bev calculation

size_payload_t = 222;
size_payload_fixed_d = 15;
size_gps_pos = 11;
size_NED_pos = 8;
size_behavior = 8;

for i=1:length(dives(:,1))  
    nb_bev(i,1) = length(dives(i,1).var.B.data(:,1));
    
    if nb_bev(i,1) > 8
        nb_bev(i,1)  = 8;
    end
    nb_pts_max(i,1) = floor((size_payload_t-size_payload_fixed_d-size_gps_pos-(nb_bev(i,1)*size_behavior))/(size_NED_pos));
end



%% Compression  


for i=1:length(dives(:,1))  

nb_pts = length(P_tab{i}(:,1));

if dive_length(i,1) > 2

% % Compression dougla Pecker    
P_reduced{i,1}(:,:) = P_tab{i,1}(:,1:3);
epsilon = 0.1;
% 
if nb_pts > nb_pts_max(i,1) 
% 
% 
% 
tic
while nb_pts > nb_pts_max(i,1)
    tic
    P_r = simplifyPolyline(P_reduced{i,1}(:,:), epsilon);
    nb_pts = length(P_r(:,1));
    epsilon = epsilon + 0.1;
end
time = toc;    
timing(i,1) = time;
P_red{i,1} = P_r;


tic
%eps_vv = 1e-2; %1e-2;
[P_red{i,3}(:,1),P_red{i,3}(:,2),P_red{i,3}(:,3)] = Visvalingam3D(P_reduced{i,1}(:,1),P_reduced{i,1}(:,2),P_reduced{i,1}(:,3),nb_pts_max(i,1));
time = toc;
timing(i,3) = time;

else
P_red{i,1}(:,:) = P_tab{i,1}(:,:);
end

% Find time after compression DP
inc_x = 1;
for j=1:length(P_red{i,1}(:,1))
    
    for jj=1:length(P_tab{i,1}(:,1))
    
    if P_red{i,1}(j,1) == P_tab{i,1}(jj,1)
        P_red{i,1}(j,4) = jj;
        jj = jj+1;
    end
    
    end
end


if ~isempty(P_red{i,3})
    
% Find time after compression VV
inc_x = 1;
for j=1:length(P_red{i,3}(:,1))
    
    for jj=1:length(P_tab{i,1}(:,1))
    
    if P_red{i,3}(j,1) == P_tab{i,1}(jj,1)
        P_red{i,3}(j,4) = jj;
        jj = jj+1;
    end
    
    end
end

end


% Compression nth point
l_step = ceil(dive_length(i,1)/ 55);
%l_step = ceil(dive_length(i,1)/ nb_pts_max(i,1));
inc = 1;
tic
for j=1:l_step:dive_length(i,1)-2
    P_red{i,2}(inc,1:2) = P_tab{i,1}(j,1:2);
    P_red{i,2}(inc,3) = P_tab{i,1}(j,3);
    P_red{i,2}(inc,4) = j;
    inc = inc+1;
end
time = toc;
timing(i,2) = time;

if rem(dive_length(i,1)-3,l_step) ~= 0
    P_red{i,2}(inc,1:2) = P_tab{i,1}(dive_length(i,1)-2,1:2);
    P_red{i,2}(inc,4) = dive_length(i,1)-2;
end

% P_red{i,2}(inc,1:2) = P_tab{i,1}(dive_length(i,1)-2,:);
% P_red{i,2}(inc,3) = dive_length(i,1)-2;

end

end

%% Adding Depth


for i=1:length(dives(:,1))
    
    if isempty(P_red{i,1})
    i=i+1;
    end
        
    for j=1:2
        
        for jj=1:length(P_red{i,j}(:,1))
            
            t_z = P_red{i,j}(jj,4);
            
%             P_red{i,j}(jj,3) =  P_tab{i,1}(t_z,3);
            
            P_z_buff1{i,j}(jj,1) = t_z;
            P_z_buff1{i,j}(jj,2) = P_red{i,j}(jj,3);
            
                
        end

    
    for jj=2:length(dives(i,1).var.B.data(:,1))
        
        if dive_length(i,1) <= 32
            i=i+1;
            
            P_z_buff1{i,j}(:,1) = P_red{i,j}(:,4);
            P_z_buff1{i,j}(:,2) = P_red{i,j}(:,3);
            P_z{i,j} = P_z_buff1{i,j};
                
        else
        
        t_z = dives(i,1).var.B.data(jj,2);
        
        P_z_buff2{i,j}(jj,1) = t_z;
        P_z_buff2{i,j}(jj,2) = P_tab{i,1}(t_z,3);
        
        P_z{i,j} = [P_z_buff1{i,j}; P_z_buff2{i,j}];
        
        end
        
        % All the dive
        [~,idx] = sort(P_z{i,j}(:,1)); % sort just the first column
        P_z_sort{i,j} =  P_z{i,j}(idx,:);
        [~,idu] = unique(P_z_sort{i,j}(:,1));
        P_z_sort{i,j} = P_z_sort{i,j}(idu,:);
        
        
        
    end
    
    end
    
end

 


%% Tortuisity

% 
% for i=1:length(dives(:,1))  
%     
% t{i,2}(1,:)= tortuosity(P_red{i,1}(:,1:2),1,1); 
% t{i,3}(1,:)= tortuosity(P_red{i,2}(:,1:2),1,1);
% 
% end


%% Rescale Trajectory for comparison

for i=2:length(dives(:,1))
    
if dive_length(i,1) <= 32
    i=i+1;
else
%     
% if length(P_red{i,1}(:,1)) < 2 || length(P_red{i,2}(:,1)) < 2
%     i=i+1;
% end


% re-interpolation
% DP
P_red_r{i,1}(:,1) = interp1(P_red{i,1}(:,end),P_red{i,1}(:,1), 1:length(P_tab{i,1}(:,1)),  'linear');
P_red_r{i,1}(:,2) = interp1(P_red{i,1}(:,end),P_red{i,1}(:,2), 1:length(P_tab{i,1}(:,1)),  'linear');
P_red_r{i,1}(:,3) = interp1(P_red{i,1}(:,end),P_red{i,1}(:,3), 1:length(P_tab{i,1}(:,1)),  'linear');
% Nth Point
P_red_r{i,2}(:,1) = interp1(P_red{i,2}(:,end),P_red{i,2}(:,1), 1:length(P_tab{i,1}(1:end-1,1)),  'linear'); %,'extrap');
P_red_r{i,2}(:,2) = interp1(P_red{i,2}(:,end),P_red{i,2}(:,2), 1:length(P_tab{i,1}(1:end-1,1)),  'linear');
P_red_r{i,2}(:,3) = interp1(P_red{i,2}(:,end),P_red{i,2}(:,3), 1:length(P_tab{i,1}(1:end-1,1)),  'linear');
% Visvalingam
P_red_r{i,3}(:,1) = interp1(P_red{i,3}(:,end),P_red{i,3}(:,1), 1:length(P_tab{i,1}(:,1)),  'linear'); %,'extrap');
P_red_r{i,3}(:,2) = interp1(P_red{i,3}(:,end),P_red{i,3}(:,2), 1:length(P_tab{i,1}(:,1)),  'linear');
P_red_r{i,3}(:,3) = interp1(P_red{i,3}(:,end),P_red{i,3}(:,3), 1:length(P_tab{i,1}(:,1)),  'linear');

end

end

%% Plot

% 
% figure(5)
% dive_nb = 55;plot(P_tab{dive_nb, 1}(:,1))
% hold on
% plot(P_red_r{dive_nb, 1}(:,1))
% plot(P_red_r{dive_nb, 2}(:,1))
% 
% 
% a1 = (P_red_r{dive_nb,1}(:,1) - P_tab{dive_nb,1}(:,1));
% a2 = (P_red_r{dive_nb,1}(:,2) - P_tab{dive_nb,1}(:,2));
% b1 = (P_red_r{dive_nb,2}(1:end-1,1) - P_tab{dive_nb,1}(1:end-2,1));
% b2 = (P_red_r{dive_nb,2}(1:end-1,2) - P_tab{dive_nb,1}(1:end-2,2));
% figure(6)
% plot(a1)
% hold on
% plot(b1)

%% 2DRMS

P_red_r{63,1}(1,:) = 0;

for i=3:length(dives(:,1))  


if ~isempty(P_red_r{i,1})

DTW{i,1} = dtw([P_red_r{i,1}(:,1)';P_red_r{i,1}(:,2)'],[P_tab{i,1}(:,1)';P_tab{i,1}(:,2)']);
DTW{i,2} = dtw([P_red_r{i,2}(:,1)';P_red_r{i,2}(:,2)'],[P_tab{i,1}(1:end-1,1)';P_tab{i,1}(1:end-1,2)']);

% DTW{i,1} = dtw([P_red{i,1}(:,1),P_red{i,1}(:,2)],[P_tab{i,1}(:,1),P_tab{i,1}(:,2)]);
% DTW{i,2} = dtw([P_red{i,2}(1:end-1,1),P_red{i,2}(1:end-1,2)],[P_tab{i,1}(1:end-2,1),P_tab{i,1}(1:end-2,2)]);
Hausdorf{i,1} = HausdorffDist([P_red_r{i,1}(:,1),P_red_r{i,1}(:,2),P_red_r{i,1}(:,3)],[P_tab{i,1}(:,1),P_tab{i,1}(:,2),P_tab{i,1}(:,3)]);
Hausdorf{i,2} = HausdorffDist([P_red_r{i,2}(1:end-1,1),P_red_r{i,2}(1:end-1,2),P_red_r{i,2}(1:end-1,3)],[P_tab{i,1}(1:end-2,1),P_tab{i,1}(1:end-2,2),P_tab{i,1}(1:end-2,3)]);
Hausdorf{i,3} = HausdorffDist([P_red_r{i,3}(1:end-1,1),P_red_r{i,3}(1:end-1,2),P_red_r{i,3}(1:end-1,3)],[P_tab{i,1}(1:end-2,1),P_tab{i,1}(1:end-2,2),P_tab{i,1}(1:end-2,3)]);

Hausdorf_z{i,1} = HausdorffDist(P_red_r{i,1}(:,3),P_tab{i,1}(:,3));
Hausdorf_z{i,2} = HausdorffDist(P_red_r{i,2}(1:end-1,3),P_tab{i,1}(1:end-2,3));
Hausdorf_z{i,3} = HausdorffDist(P_red_r{i,3}(1:end-1,3),P_tab{i,1}(1:end-2,3));


Frechet{i,1} = DiscreteFrechetDist([P_red_r{i,1}(:,1),P_red_r{i,1}(:,2)],[P_tab{i,1}(:,1),P_tab{i,1}(:,2)]);
Frechet{i,2} = DiscreteFrechetDist([P_red_r{i,2}(1:end-1,1),P_red_r{i,2}(1:end-1,2)],[P_tab{i,1}(1:end-2,1),P_tab{i,1}(1:end-2,2)]);

%     len = length(P_tab{i,1}(:,1));
DRMS{i,1} = 2*sqrt(mean((P_red_r{i,1}(:,1) - P_tab{i,1}(:,1)).^2+(P_red_r{i,1}(:,2) - P_tab{i,1}(:,2)).^2));
DRMS{i,2} = 2*sqrt(mean((P_red_r{i,2}(1:end-1,1) - P_tab{i,1}(1:end-2,1)).^2+(P_red_r{i,2}(1:end-1,2) - P_tab{i,1}(1:end-2,2)).^2));
% DRMS{i,1} = 2*sqrt(mean(abs(P_red_r{i,1}(:,1) - P_tab{i,1}(:,1))+ abs(P_red_r{i,1}(:,2) - P_tab{i,1}(:,2))));
% DRMS{i,2} = 2*sqrt(mean(abs(P_red_r{i,2}(1:end-1,1) - P_tab{i,1}(1:end-2,1))+abs(P_red_r{i,2}(1:end-1,2) - P_tab{i,1}(1:end-2,2))));
% DRMS{i,1} = 2*sqrt(mean((P_red_r{i,1}(1:len,1) - P_tab{i,1}(:,1)).^2+(P_red_r{i,1}(1:len,2) - P_tab{i,1}(:,2)).^2));
% DRMS{i,2} = 2*sqrt(mean((P_red_r{i,2}(1:len,1) - P_tab{i,1}(:,1)).^2+(P_red_r{i,2}(1:len,2) - P_tab{i,1}(:,2)).^2));

DRMS{i,3} = dive_length(i,1);
Hausdorf{i,4} = dive_length(i,1);
Frechet{i,4} = dive_length(i,1);
end

end

%% Store all the dive variables
inc = 1;
for i=1:length(dives(:,1))  
    
if ~isempty(DRMS{i,1})   
   dive_var(inc,1) = i; % Dive_nb
   dive_var(inc,2) = dive_length(i,1); % Dive_length
   dive_var(inc,3) = dive_type(i,1);
   dive_var(inc,4) = t{i,1}(1,1);
   dive_var(inc,5) = Hausdorf{i,1};  
   dive_var(inc,6) = Hausdorf{i,2};  
   dive_var(inc,7) = (Hausdorf{i,2}-Hausdorf{i,1})/Hausdorf{i,2}; 
   dive_var(inc,8) = timing(i,1); 
   dive_var(inc,9) = timing(i,2);  
   inc = inc+1;
end

end

% Without rest
inc = 1;
inc2= 1;
for i=1:length(dives(:,1))   
    if ~isempty(DRMS{i,1})
        if dive_type(i,1) ~= 5
            dive_var_nr(inc,1) = i; % Dive_nb
            dive_var_nr(inc,2) = dive_length(i,1); % Dive_length
            dive_var_nr(inc,3) = dive_type(i,1);
            dive_var_nr(inc,4) = t{i,1}(1,1);
            dive_var_nr(inc,5) = Hausdorf{i,1};
            dive_var_nr(inc,6) = Hausdorf{i,2};
            dive_var_nr(inc,7) = Hausdorf{i,3};
            dive_var_nr(inc,8) = Hausdorf_z{i,1};
            dive_var_nr(inc,9) = Hausdorf_z{i,2};
            dive_var_nr(inc,10) = Hausdorf_z{i,3};
            dive_var_nr(inc,11) = timing(i,1); 
            dive_var_nr(inc,12) = timing(i,2);
            dive_var_nr(inc,13) = timing(i,3);
            inc = inc+1;
        else
            dive_var_r(inc2,1) = i; % Dive_nb
            dive_var_r(inc2,2) = dive_length(i,1); % Dive_length
            dive_var_r(inc2,3) = dive_type(i,1);
            dive_var_r(inc2,4) = t{i,1}(1,1);
            dive_var_r(inc2,5) = Hausdorf{i,1};
            dive_var_r(inc2,6) = Hausdorf{i,2};
            dive_var_r(inc2,7) = Hausdorf{i,3};
            dive_var_r(inc2,8) = Hausdorf_z{i,1};
            dive_var_r(inc2,9) = Hausdorf_z{i,2};
            dive_var_r(inc2,10) = Hausdorf_z{i,3};            dive_var_r(inc2,11) = timing(i,1); 
            dive_var_r(inc2,12) = timing(i,2);
            dive_var_r(inc2,13) = timing(i,3);
            inc2 = inc2+1;          
        end       
    end
end

% Long and short dive
inc = 1;
inc2= 1;
for i=1:length(dives(:,1))   
    if ~isempty(DRMS{i,1})
        if dive_type(i,1) == 3 || dive_type(i,1) == 4 
            dive_var_l(inc,1) = i; % Dive_nb
            dive_var_l(inc,2) = dive_length(i,1); % Dive_length
            dive_var_l(inc,3) = dive_type(i,1);
            dive_var_l(inc,4) = t{i,1}(1,1);
            dive_var_l(inc,5) = Hausdorf{i,1};
            dive_var_l(inc,6) = Hausdorf{i,2};
            dive_var_l(inc,7) = Hausdorf{i,3};
            dive_var_l(inc,8) = Hausdorf_z{i,1};
            dive_var_l(inc,9) = Hausdorf_z{i,2};
            dive_var_l(inc,10) = Hausdorf_z{i,3};
            dive_var_l(inc,11) = timing(i,1); 
            dive_var_l(inc,12) = timing(i,2);
            dive_var_l(inc,13) = timing(i,3);
            inc = inc+1;
        elseif dive_type(i,1) == 2
            dive_var_s(inc2,1) = i; % Dive_nb
            dive_var_s(inc2,2) = dive_length(i,1); % Dive_length
            dive_var_s(inc2,3) = dive_type(i,1);
            dive_var_s(inc2,4) = t{i,1}(1,1);
            dive_var_s(inc2,5) = Hausdorf{i,1};
            dive_var_s(inc2,6) = Hausdorf{i,2};
            dive_var_s(inc2,7) = Hausdorf{i,3};
            dive_var_s(inc2,8) = Hausdorf_z{i,1};
            dive_var_s(inc2,9) = Hausdorf_z{i,2};
            dive_var_s(inc2,10) = Hausdorf_z{i,3};
            dive_var_s(inc2,11) = timing(i,1); 
            dive_var_s(inc2,12) = timing(i,2);
            dive_var_s(inc2,13) = timing(i,3);
            inc2 = inc2+1;          
        end       
    end
end


%% Plot dive
inc_plot=10;
figure(inc_plot)
dive_nb =14;
plot(P_tab{dive_nb, 1}(:,1),P_tab{dive_nb, 1}(:,2),'LineWidth',2)
hold on
plot(P_red_r{dive_nb, 1}(:,1),P_red_r{dive_nb, 1}(:,2),'LineWidth',2)
plot(P_red_r{dive_nb, 2}(:,1),P_red_r{dive_nb, 2}(:,2),'LineWidth',2)
Title = sprintf("Dive N°%d, Duration:%.0fs , Tortusity:%.2f", dive_nb(1,1), dive_length(dive_nb,1), t{dive_nb,1}(1,1));
%"Dive N°" + dive_nb(1,1) + ", length:"+ dive_length(dive_nb,1) +", Tortusity:"+  t{dive_nb,1};
title(Title(:,1),'FontWeight','bold',...
    'FontSize',14);
legend('Reference',sprintf('DP, HD: %.2f, Nb Point : %.0f', Hausdorf{dive_nb,1}, length(P_red{dive_nb,1}(:,1))), sprintf('NP, HD: %.2f, Nb Point : %.0f', Hausdorf{dive_nb,2},length(P_red{dive_nb,2}(:,1))),'FontWeight','bold',...
    'FontSize',14);
xlabel('Eastward (m)','FontWeight','bold','FontSize',14);
ylabel('Northward (m)','FontWeight','bold','FontSize',14);
inc_plot = inc_plot+1;
axis equal
hold off

figure(inc_plot+1)
plot(-P_tab{dive_nb, 1}(:,3),'LineWidth',2)
hold on
plot(-P_red_r{dive_nb, 1}(:,3),'LineWidth',2)
plot(-P_red_r{dive_nb, 2}(:,3),'LineWidth',2)
Title = sprintf("Dive N°%d, Duration:%.0fs , Tortusity:%.2f", dive_nb(1,1), dive_length(dive_nb,1), t{dive_nb,1}(1,1));
%"Dive N°" + dive_nb(1,1) + ", length:"+ dive_length(dive_nb,1) +", Tortusity:"+  t{dive_nb,1};
title(Title(:,1),'FontWeight','bold',...
    'FontSize',14);
legend('Reference',sprintf('DP, HD: %.2f, Nb Point : %.0f', Hausdorf{dive_nb,1}, length(P_red{dive_nb,1}(:,1))), sprintf('NP, HD: %.2f, Nb Point : %.0f', Hausdorf{dive_nb,2},length(P_red{dive_nb,2}(:,1))),'FontWeight','bold',...
    'FontSize',14);
xlabel('time (s)','FontWeight','bold','FontSize',14);
ylabel('Depth (m)','FontWeight','bold','FontSize',14);
hold off




%% Analyse length / 2DRMS

% All the dive
[~,idx] = sort((dive_var_nr(:,1))); % sort just the first column
dive_var_sort_nr = dive_var_nr(idx,:);  

x = dive_var_sort_nr(:,2);
y = dive_var_sort_nr(:,6);

y1 = 0.002631*x + -0.09678;
y2 = 0.007166*x + -0.4121;

figure(7)
subplot(2,2,1)
plot(dive_var_nr(:,2),dive_var_nr(:,5),'xr','LineWidth',2)
hold on
plot(dive_var_nr(:,2),dive_var_nr(:,6),'xb','LineWidth',2)
plot(dive_var_r(:,2),dive_var_r(:,5),'xr','LineWidth',2)
plot(dive_var_r(:,2),dive_var_r(:,6),'xb','LineWidth',2)


plot(x,y1,'r')
plot(x,y2,'b')
Title = sprintf(" A) Dives (without REST) duration in function of HD");
%"Dive N°" + dive_nb(1,1) + ", length:"+ dive_length(dive_nb,1) +", Tortusity:"+  t{dive_nb,1};
title(Title,'FontWeight','bold','FontSize',14);
legend('DP','Nth Point', 'Fitted function for DP','Fitted function for Nth algorithm','FontWeight','bold',...
    'FontSize',14);
xlabel('Dive duration (s)','FontWeight','bold','FontSize',14);
ylabel('Hausdorff distance (m)','FontWeight','bold','FontSize',14);


subplot(2,2,2)
plot(dive_var_nr(:,4),dive_var_nr(:,5),'xr','LineWidth',2)
hold on
plot(dive_var_nr(:,4),dive_var_nr(:,6),'xb','LineWidth',2)
plot(dive_var_r(:,4),dive_var_r(:,5),'xr','LineWidth',2)
plot(dive_var_r(:,4),dive_var_r(:,6),'xb','LineWidth',2)

Title = sprintf(" B) Dives (without REST) tortuosity in function of HD");
%"Dive N°" + dive_nb(1,1) + ", length:"+ dive_length(dive_nb,1) +", Tortusity:"+  t{dive_nb,1};
title(Title,'FontWeight','bold','FontSize',14);
legend('DP','Nth Point','FontWeight','bold',...
    'FontSize',14);
xlabel('Tortuosity','FontWeight','bold','FontSize',14);
ylabel('Hausdorff distance (m)','FontWeight','bold','FontSize',14);
ylim([-1 8])
% Removing Dive with Rest




%figure(8)
subplot(2,2,3)
plot(dive_var_nr(:,2),dive_var_nr(:,11),'xr','LineWidth',2)
hold on
plot(dive_var_nr(:,2),dive_var_nr(:,12),'xb','LineWidth',2)
plot(dive_var_r(:,2),dive_var_r(:,12),'xb','LineWidth',2)
plot(dive_var_r(:,2),dive_var_r(:,11),'xr','LineWidth',2)

Title = sprintf(" C) Dives (without REST) duration in function of HD");
%"Dive N°" + dive_nb(1,1) + ", length:"+ dive_length(dive_nb,1) +", Tortusity:"+  t{dive_nb,1};
title(Title,'FontWeight','bold','FontSize',14);
legend('DP','Nth Point','FontWeight','bold',...
    'FontSize',14);
xlabel('Dive duration (s)','FontWeight','bold','FontSize',14);
ylabel('Compute time (s)','FontWeight','bold','FontSize',14);


subplot(2,2,4)
plot(dive_var_nr(:,4),dive_var_nr(:,11),'xr','LineWidth',2)
hold on
plot(dive_var_nr(:,4),dive_var_nr(:,12),'xb','LineWidth',2)
plot(dive_var_r(:,4),dive_var_r(:,12),'xb','LineWidth',2)
plot(dive_var_r(:,4),dive_var_r(:,11),'xr','LineWidth',2)
Title = sprintf(" D) Dives (without REST) tortuosity in function of HD");
%"Dive N°" + dive_nb(1,1) + ", length:"+ dive_length(dive_nb,1) +", Tortusity:"+  t{dive_nb,1};
title(Title,'FontWeight','bold','FontSize',14);
legend('DP','Nth Point','FontWeight','bold',...
    'FontSize',14);
xlabel('Tortuosity','FontWeight','bold','FontSize',14);
ylabel('Compute time (s)','FontWeight','bold','FontSize',14);
% Removing Dive with Rest


%% Tortusity / 2DRMS
figure(5)
plot(dive_var_nr(:,4),dive_var_nr(:,5),'xr')
%plot(dive_var_r(:,4),dive_var_r(:,5),'xr')
hold on
plot(dive_var_nr(:,4),dive_var_nr(:,6),'xb')
%plot(dive_var_r(:,4),dive_var_r(:,6),'xb')

%plot3(dive_var_nr(:,2),dive_var_nr(:,5),dive_var_nr(:,4));

%% Tortusity / length
figure(6)
plot(dive_var_nr(:,4),dive_var_nr(:,2),'xr')
%plot(dive_var_r(:,4),dive_var_r(:,5),'xr')


%plot3(dive_var_nr(:,2),dive_var_nr(:,5),dive_var_nr(:,4));


% Average Frechet / Hausdorf / DTW per class

avg_hf(1,:) = mean(dive_var_s(:,5:13));
avg_hf(2,:) = mean(dive_var_l(:,5:13));
avg_hf(3,:) = mean(dive_var_nr(:,5:13));
avg_hf(4,:) = mean(dive_var_r(:,5:13));

