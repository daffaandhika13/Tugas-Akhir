clc
close all
clear all

%IMPORTANT
% install Marine Systems Simulator (MSS) from add-on explorer
% or 
% install MSS from github (README.md)





%% Module
    sim_time=1000;         % simulation time limit in seconds default 5000 
    % 1 Heading Controller
    % 2 Waypoint Guidance

%% Heading Controller
    %initial heading
    init_heading        =-30;    % Initial heading in degrees 0 - 360 
    desired_heading     =180;    % Desired heading in degrees 0 - 360   
    
    %Heading controller PID
    H_control_pid=[50 0 0];

    phithepsi0=[0 0 init_heading*pi/180];         %(rad)


    controller = 2; % 1 = PID ; 2 = PID FUZZY
%% Waypoint Guidance
    %Guidance Gain 
    Gpi=[2 0.1]; %[Pterm Iterm]
    gamma=0.01; %ALOS onnly
    delta=25; %lookahead distance
    
    %select guidance
    guidance = 2;
    % 1 PLOS  >>Set Gpi Pterm only
    % 2 ILOS >>Set Gpi Pterm and Iterm
    % 3 ALOS >>Set Gpi Pterm and adaptive gain gamma 
    

    R_switch=1; %Distance to switch to next waypoint     

    
%% Waypoint
num_points = 100;  % Jumlah waypoint yang diinginkan
x_waypoint = zeros(num_points, 1);  % Inisialisasi X waypoint
y_waypoint = zeros(num_points, 1);  % Inisialisasi Y waypoint
% Inisialisasi titik pertama
x_waypoint(1) = 5;  % Titik pertama X
y_waypoint(1) = 5;  % Titik pertama Y

% Loop untuk menghasilkan waypoint yang membuat USV bisa kembali ke x sebelumnya
for i = 2:num_points
    % Pola untuk x_waypoint yang sangat acak, dengan mundur untuk pergerakan drastis
    if mod(i, 4) == 0
        x_waypoint(i) = x_waypoint(i-1) - (150 + mod(i*73, 300));  % Mundur drastis
    elseif mod(i, 6) == 0
        x_waypoint(i) = x_waypoint(i-1) + (300 + mod(i*97, 200));  % Lompatan besar ke depan
    elseif mod(i, 8) == 0
        x_waypoint(i) = x_waypoint(i-1) - (200 + mod(i*59, 250));  % Mundur besar lagi
    else
        x_waypoint(i) = x_waypoint(i-1) + (100 + mod(i*47, 100));  % Lompatan biasa ke depan
    end
    
    % Variasi y_waypoint yang juga cukup drastis
    if mod(i, 3) == 0
        y_waypoint(i) = y_waypoint(i-1) + (100 + mod(i*33, 150));  % Lompatan besar ke atas
    elseif mod(i, 5) == 0
        y_waypoint(i) = y_waypoint(i-1) - (120 + mod(i*43, 170));  % Lompatan besar ke bawah
    elseif mod(i, 7) == 0
        y_waypoint(i) = y_waypoint(i-1) + (200 + mod(i*51, 180));  % Lompatan lebih besar lagi ke atas
    else
        y_waypoint(i) = y_waypoint(i-1) + (50 + mod(i*27, 100));   % Lompatan biasa
    end
end

% Parameter lingkaran
radius = 20; % Radius lingkaran
n_points = 100; % Jumlah waypoint untuk membentuk lingkaran

% Buat theta untuk membentuk lingkaran penuh
theta = linspace(0, 2*pi, n_points);

% Hitung waypoint untuk lingkaran
x_ling = radius * cos(theta);
y_ling = radius * sin(theta);

% Pastikan waypoint dimulai dari (0, 0)
x_ling = x_ling - radius; % Geser semua nilai X agar dimulai dari 0
x_ling = abs(x_ling);
% Plot lingkaran
x_8 = [x_ling(1:50) x_ling(1:50)+40 x_ling(51:100)+40 x_ling(51:100)];
y_8 = [y_ling y_ling];
% plot(x_8,y_8);
x_elips = 40 * cos(theta);
y_elips = 20 * sin(theta);

x_custom = [0 3 6 9 12 15 18 21 24 27 30 33 36 33 30 27 24 21 18 15 12 9 6 3 0];
y_custom = [0 5 8 12 15 17 18 17 15 12 8 5 0 -5 -8 -12 -15 -17 -18 -17 -15 -12 -8 -5 0];

% Pastikan waypoint dimulai dari (0, 0)
x_elips = x_elips - 40; % Geser semua nilai X agar dimulai dari 0
x_elips = abs(x_elips);

x_8e = [x_elips(1:50) x_elips(1:50)+60 x_elips(51:100)+60 x_elips(51:100)];
y_8e = [y_elips y_elips];

%% 


%sim 'OtterUSV.slx' 
sim Simulasi_3DOFLengkap.slx 
out=ans;

xte{1}=out.xte_los; ate{1}=out.ate_los;
xte{2}=out.xte_ilos; ate{2}=out.ate_ilos;
xte{3}=out.xte_alos; ate{3}=out.ate_alos;



%% plotting
%%
xlimm(1)=min(out.posisi_xy(:,2))-10;
xlimm(2)=max(out.posisi_xy(:,2))+10;
ylimm(1)=min(out.posisi_xy(:,1))-10;
ylimm(2)=max(out.posisi_xy(:,1))+10;
xlimm=[xlimm(1) xlimm(1) xlimm(2) xlimm(2) xlimm(1)];
ylimm=[ylimm(1) ylimm(2) ylimm(2) ylimm(1) ylimm(1)];
xlimm=xlimm; ylimm=ylimm;
x_waypoint = out.Waypoint_y;
y_waypoint = out.Waypoint_x;

close all

    run error_area.m
    subtitle(strcat('Error area = ',num2str(errorArea)));
    %run plot_wp.m   
    fig = figure('Units','normalized');
    ax = axes();
    plot( out.posisi_xy(:,1),out.posisi_xy(:,2),y_waypoint(1,1:end),x_waypoint(1,1:end), 'LineWidth',2, 'MarkerSize',4);
    legend("USV","Waypoint");
    title('USV Trajectory and Heading');
    subtitle('arrow indicates USV direction')
    xlabel('m');
    ylabel('m');
    pos = ax.Position;
    pos(3:4) = pos(3:4) + pos(1:2);
    % for jj=1:200:length(realData)
    %     arrow=[0 0; 1 50];
    %     psi=out.act_head.Data;
    %     R=[cos(psi(jj)) -sin(psi(jj));sin(psi(jj)) cos(psi(jj))];
    %     arrow=R'*arrow;
    %     arrow_x_fig = interp1(ax.XLim, pos([1 3]), arrow(1,:)+realData(jj,1));
    %     arrow_y_fig = interp1(ax.YLim, pos([2 4]), arrow(2,:)+realData(jj,2));
    %     annotation('arrow', arrow_x_fig, arrow_y_fig,'LineStyle','-',HeadWidth=2,HeadLength=2);
    % end
   
    % Heading
    figure()
    subplot(2,1,1)
        plot(out.guid_out*180/pi,LineWidth=2);title('Heading');hold on
        plot(out.act_head*180/pi,LineWidth=2,LineStyle='--');
        xlabel('time(s)');ylabel('Heading (deg)');grid minor;legend('guidance','actual')
    subplot(2,1,2)
        herrmse=out.err_head.Data;
        herrmse=sqrt(mean((herrmse).^2));
        plot (out.err_head,LineWidth=2);title('Heading Error');
        subtitle(strcat('RMSE = ',num2str(herrmse)));
        xlabel('time(s)');ylabel('error (deg)');grid minor
        hold on;


    % Cross Track Error
    ctermse=xte{guidance};
    ctermse=sqrt(mean((ctermse.Data).^2));
    figure()
    plot (xte{guidance},LineWidth=2);title('Cross Track Error');
    subtitle(strcat('RMSE =',num2str(ctermse)));
    xlabel('time(s)');ylabel('error (m)');grid minor
    hold on;    
hold off
%% 
% Data kecepatan (contoh data)
Vx = out.kecepatan(:,1); % Kecepatan x (contoh: fungsi sinus)
Vy = out.kecepatan(:,2); % Kecepatan y (contoh: fungsi cosinus)
Vyaw =out.kecepatan(:,3); % Kecepatan yaw (contoh: fungsi linear)
% Plot data kecepatan
figure; % Membuka jendela figure baru
plot(Vx, 'r-', 'LineWidth', 1.5); % Plot Vx dengan garis merah
hold on; % Mempertahankan plot agar Vy dan Vyaw bisa ditambahkan
% plot(Vy, 'b--', 'LineWidth', 1.5); % Plot Vy dengan garis biru putus-putus
% plot(Vyaw, 'g-.', 'LineWidth', 1.5); % Plot Vyaw dengan garis hijau titik-dash

% Tambahkan judul, label sumbu, dan legenda
title('Velocity');
xlabel('Time (seconds)');
ylabel('Velocity');
% legend('Vx (x-axis velocity)', 'Vy (y-axis velocity)', 'Vyaw (yaw velocity)', 'Location', 'best');
legend('Vx (x-axis velocity)', 'Location', 'best');

figure(6)
hold on; % Keep both plots in the same figure
plot(out.vref, 'r', 'DisplayName', 'vref'); % Red color for vref with legend
plot(out.vactual, 'b', 'DisplayName', 'vactual'); % Blue color for vactual with legend
hold off; % Release the hold after plotting

figure(7)
plot(out.f_rudder);
% Adding legend
legend show;

% Menyesuaikan tampilan
% grid on; % Menampilkan grid

