
clear;clc

a = xlsread("data3/dtt1_a.csv");
b = xlsread("data3/dtt1_b.csv");
c = xlsread("data3/dtt1_c.csv");
d = xlsread("data3/dtt1_d.csv");
s = xlsread("data3/dtt1_s.csv");
q = xlsread("data3/dtt1_q.csv");
qv = xlsread("data3/dtt1_qv.csv");
qa = xlsread("data3/dtt1_qa.csv");
yaw=xlsread("data3/dtt1_yaw.csv");

% curvature velocity limit
curve = zeros(length(q), 1);
curve_max=-100;
curve_min=100;
for i = 1 : length(q)
    j = max(min(i, length(q)-1),2);
    curve_dist = [-norm(q(j,:)-q(j-1,:));0;norm(q(j,:)-q(j+1,:))];
    curve_mat = [ones(3,1), curve_dist, curve_dist.^2];
    curve_x = [q(j-1,1); q(j,1); q(j+1,1)];
    curve_y = [q(j-1,2); q(j,2); q(j+1,2)];
    curve_px = curve_mat \ curve_x;
    curve_py = curve_mat \ curve_y;
    curve(i) = -2.0*(curve_px(3)*curve_py(2)-curve_py(3)*curve_px(2))...
        /(curve_px(2)^2+curve_py(2)^2)^(3/2);
    curve_max=max(curve(i),curve_max);
    curve_min=min(curve(i),curve_min);
end
acc_max = 1.0; %note
v_curve_max = sqrt(acc_max./abs(curve));

% origin method velocity limit
v_origin_max = ones(length(q), 1)*1e20;
v_origin_min = zeros(length(q), 1);
for i = 1:length(q)-1
    if(qa(i,1) > 0)
        v_origin_max(i) = min(v_origin_max(i), sqrt(max((acc_max-qv(i,1)*a(i))/qa(i,1),0)));
    else
        v_origin_min(i) = max(v_origin_min(i), sqrt(max((acc_max-qv(i,1)*a(i))/qa(i,1),0)));
    end
    if(qa(i,2) > 0)
        v_origin_max(i) = min(v_origin_max(i), sqrt(max((acc_max-qv(i,2)*a(i))/qa(i,2),0)));
    else
        v_origin_min(i) = max(v_origin_min(i), sqrt(max((acc_max-qv(i,2)*a(i))/qa(i,2),0)));
    end
end


figure(1); clf(1);
plot(q(:,1), q(:,2), '-b','LineWidth',3);
% axis equal;
grid on;
xlabel('X(m)','FontName','Times New Roman','FontSize',14);
ylabel('Y(m)','FontName','Times New Roman','FontSize',14);
legend({ '规划轨迹'},'FontSize',14);
% title('global trajectory');

figure(2); clf(2);
ds=zeros(length(s),1);
dt=zeros(length(s),1);
for i=2:length(s)
    ds(i)=s(i)-s(i-1);
end
    
T=sum(sqrt(b)./ds)
for i=2:length(s)
    dt(i)=ds(i)/sqrt(b(i));
end
% plot(s, v_origin_max, '-', 'LineWidth', 3, 'Color', [0.8,0.5,0.5]);
v_max=ones(length(s),1);
plot(s, v_max, '-', 'LineWidth', 3, 'Color', [0.8,0.3,0.8]);
hold on;
% plot(s, v_origin_min, '-.', 'LineWidth', 3, 'Color', [0.8,0.5,0.5]);
v_min=zeros(length(s),1);
plot(s, v_min, '-', 'LineWidth', 3, 'Color', [0.8,0.5,0.5]);
hold on;
plot(s, sqrt(b), '-b', 'LineWidth', 2.0);
xlabel('s(m)','FontName','Times New Roman','FontSize',14);
ylabel('v(m/s)','FontName','Times New Roman','FontSize',14);
legend({'规划速度上界','规划速度下界','规划速度'},'FontSize',14);
grid on; 
ylim([0, 1.5]);

% 曲率
figure(3);clf(3);


curv_plot_max=ones(length(curve),1)*curve_max;
curv_plot_min=ones(length(curve),1)*curve_min;
plot(s(1:end-1), curve, '-', 'LineWidth', 2);
hold on ;
plot(s(1:end-1), curv_plot_max, ':g', 'LineWidth', 2,'Color', [0.8,0.3,0.8]);
hold on 
plot(s(1:end-1), curv_plot_min, ':g', 'LineWidth', 2,'Color', [0.8,0.5,0.5]);

xlabel('s(m)','FontName','Times New Roman','FontSize',14);
ylabel('1/R(1/m)','FontSize',14);
legend({'规划曲率','规划曲率上界','规划曲率下界'},'FontSize',14);


figure(4);clf(4);
plot(s(1:end-1), a, '-r', 'LineWidth', 2);
hold on;
a_max=ones(length(s)-1,1)*0.6;
plot(s(1:end-1), a_max, ':g', 'LineWidth', 2,'Color', [0.8,0.3,0.8]);
hold on;
a_min=ones(length(s)-1,1)*-0.6;
plot(s(1:end-1),a_min, ':g', 'LineWidth', 2,'Color', [0.8,0.5,0.5]);
xlabel('s(m)','FontName','Times New Roman','FontSize',14);
% yyaxis right;
ylabel('a(m/s^2)','FontSize',14);
grid on; 
legend({'规划加速度','规划加速度上界','规划加速度下界'},'FontSize',14);
ylim([-0.7, 1.2]);
% hsl(150, 16.51%, 57.25%)

figure(5);clf(5);
plot(s(1:length(yaw)),yaw*180/pi, 'LineWidth', 2,'Color',[0.2,0.22,0.15]);
xlabel('s(m)','FontName','Times New Roman','FontSize',14);
ylabel('角度(deg)','FontSize',14);
grid on; 
legend({ '规划航向角'});







