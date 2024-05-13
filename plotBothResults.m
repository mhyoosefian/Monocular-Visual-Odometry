function plotBothResults

load MVO/finalRot_2d.mat;
load MVO/finalTrans_2d.mat;
finalRot2 = finalRot;       % 2 indicates that the optimization is between two frames
finalTr2 = finalTr;

load FMVO/finalRot_Optimized.mat
load FMVO/finalTrans_Optimized.mat
finalRotn = finalRot;       % n indicates that the optimization is between n frames
finalTrn = finalTr;

load MVO/GT.mat
grt_data = data;
imageIdx = 1000;
grt_data = grt_data(6 + 10*(imageIdx-23):10:end,:);

load MVO/runTime_2d.mat
iter_time2 = iter_time;
iter_time2 = iter_time2(iter_time2 ~= 0);

load FMVO/runTime_Optimized.mat
iter_timen = iter_time;
iter_timen = iter_timen(iter_timen ~= 0);



%% Orientation
% the rotation between cam and imu frames (from imu to cam)
RI0C0 = [0.014865542981800  -0.999880929698000   0.004140296794220
    0.999557249008000   0.014967213324700   0.025715529948000
    -0.025774436697400   0.003756188357970   0.999660727178000];
N = length(finalTrn);
eul2 = zeros(N, 3);
euln = zeros(N, 3);
for i=1:N
     RI02(:,:,i) = RI0C0*finalRot2(:,:,i).'*inv(RI0C0);
     RI0n(:,:,i) = RI0C0*finalRotn(:,:,i).'*inv(RI0C0);
     eul2(i,:) = rotm2eul(RI02(:,:,i));
     euln(i,:) = rotm2eul(RI0n(:,:,i));
end
phi2 = eul2(:,3);
theta2 = eul2(:,2);
psi2 = eul2(:,1);
phin = euln(:,3);
thetan = euln(:,2);
psin = euln(:,1);

grt_quat = grt_data(:,5:8);
grt_quat = grt_quat(1:N,:);
RWI0 = quat2rotm(grt_quat(1,:));

for i=1:N
    grt_RI0(:,:,i) = inv(RWI0)*quat2rotm(grt_quat(i,:)); 
%     RI0(:,:,i) = inv(RI0(:,:,1))*RI0(:,:,i); 
end
eul_true = rotm2eul(grt_RI0);


figure;
subplot(3, 1, 1)
plot(1:N,phi2*180/pi, 'b', 1:N,phin*180/pi, 'r', 1:N, eul_true(:,3)*180/pi, 'k', 'LineWidth', 1.2);
ylabel('Degrees', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend('$\phi_{MVO}$', '$\phi_{FMVO}$', '$\phi_{true}$', 'interpreter', 'latex', 'fontSize', 14);

subplot(3, 1, 2)
plot(1:N,theta2*180/pi, 'b', 1:N,thetan*180/pi, 'r', 1:N, eul_true(:,2)*180/pi, 'k', 'LineWidth', 1.2);
ylabel('Degrees', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend('$\theta_{MVO}$', '$\theta_{FMVO}$', '$\theta_{true}$','interpreter', 'latex', 'fontSize', 14);

subplot(3, 1, 3)
plot(1:N,psi2*180/pi, 'b', 1:N,psin*180/pi, 'r', 1:N, eul_true(:,1)*180/pi, 'k', 'LineWidth', 1.2);
xlabel('Sample Number', 'Interpreter', 'latex');
ylabel('Degrees', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend('$\psi_{MVO}$', '$\psi_{FMVO}$', '$\psi_{true}$', 'interpreter', 'latex', 'fontSize', 14);


%% Position
trI0C0 = [-0.0216401454975; -0.064676986768; 0.00981073058949];
grt_position = grt_data(:,2:4).';
grt_position = grt_position(:,1:N);

% Ground-truth position (of different IMU frames wrt the first IMU frame)
trWI0 = grt_position(:,1);
for i=1:N
    grt_trI0(:,i) = inv(RWI0)*(grt_position(:,i) - trWI0); 
end

for i=1:N
    trI02(:,i) = trI0C0 + RI0C0*finalTr2(:,i) - RI02(:,:,i)*trI0C0;
    trI0n(:,i) = trI0C0 + RI0C0*finalTrn(:,i) - RI0n(:,:,i)*trI0C0;
end
magnitudes2 = sqrt(sum((trI02').^2,2));
magnitudesn = sqrt(sum((trI0n').^2,2));
magnitudesGT = sqrt(sum((grt_trI0').^2,2));
scaleFactor2 = median(magnitudesGT(2:end) ./ magnitudes2(2:end));
scaleFactorn = median(magnitudesGT(2:end) ./ magnitudesn(2:end));
new2 = trI02.*scaleFactor2;
newn = trI0n.*scaleFactorn;

figure;
subplot(1, 2, 1);
plot(-new2(1,1:N),-new2(3,1:N), 'b', -newn(1,1:N),-newn(3,1:N), 'r', grt_trI0(1,1:N),grt_trI0(3,1:N), 'k', 'LineWidth', 1.2);
xlabel('x ($m$)', 'Interpreter', 'latex');
ylabel('z ($m$)', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend({'MVO', 'FMVO', 'Ground Truth'},'Location','northwest', 'interpreter', 'latex', 'FontSize', 14);

subplot(1, 2, 2);
plot(-new2(1,1:N), -new2(2,1:N), 'b', -newn(1,1:N),-newn(2,1:N), 'r', grt_trI0(1,1:N),grt_trI0(2,1:N), 'k', 'LineWidth', 1.2);
xlabel('x ($m$)', 'Interpreter', 'latex');
ylabel('y ($m$)', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend({'MVO', 'FMVO', 'Ground Truth'},'Location','northwest', 'interpreter', 'latex', 'FontSize', 14);


%% Orientation RMSE computation
rmse_phi2 = 0; rmse_theta2 = 0; rmse_psi2 = 0;
rmse_phin = 0; rmse_thetan = 0; rmse_psin = 0;
for i=1:N
    rmse_phi2(i+1) = rmse_phi2(i) + norm( (phi2(i) - eul_true(i, 3))*180/pi)^2;
    rmse_theta2(i+1) = rmse_theta2(i) + norm( (theta2(i) - eul_true(i, 2))*180/pi )^2;
    rmse_psi2(i+1) = rmse_psi2(i) + norm( (psi2(i) - eul_true(i, 1))*180/pi )^2;

    rmse_phin(i+1) = rmse_phin(i) + norm( (phin(i) - eul_true(i, 3))*180/pi )^2;
    rmse_thetan(i+1) = rmse_thetan(i) + norm( (thetan(i) - eul_true(i, 2))*180/pi )^2;
    rmse_psin(i+1) = rmse_psin(i) + norm( (psin(i) - eul_true(i, 1))*180/pi )^2;
end
rmse_phi2 = sqrt(rmse_phi2./(N+1));
rmse_theta2 = sqrt(rmse_theta2./(N+1));
rmse_psi2 = sqrt(rmse_psi2./(N+1));
rmse_phin = sqrt(rmse_phin./(N+1));
rmse_thetan = sqrt(rmse_thetan./(N+1));
rmse_psin = sqrt(rmse_psin./(N+1));

figure;
subplot(3, 1, 1)
plot(1:N+1,rmse_phi2, 'b', 1:N+1, rmse_phin, 'r', 'LineWidth', 1.2);
ylabel('Degrees', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend('RMSE($\phi_{MVO}$)', 'RMSE($\phi_{FMVO}$)', 'interpreter', 'latex', 'fontSize', 14);

subplot(3, 1, 2)
plot(1:N+1,rmse_theta2, 'b', 1:N+1, rmse_thetan, 'r', 'LineWidth', 1.2);
ylabel('Degrees', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend('RMSE($\theta_{MVO}$)', 'RMSE($\theta_{FMVO}$)', 'interpreter', 'latex', 'fontSize', 14);

subplot(3, 1, 3)
plot(1:N+1,rmse_psi2, 'b', 1:N+1, rmse_psin, 'r', 'LineWidth', 1.2);
xlabel('Sample Number', 'Interpreter', 'latex');
ylabel('Degrees', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend('RMSE($\psi_{MVO}$)', 'RMSE($\psi_{FMVO}$)', 'interpreter', 'latex', 'fontSize', 14);



disp(['Median RMSE of phi for MVO is ', num2str(median(rmse_phi2))]);
disp(['Median RMSE of theta for MVO is ', num2str(median(rmse_theta2))]);
disp(['Median RMSE of psi for MVO is ', num2str(median(rmse_psi2))]);

disp(['Median RMSE of phi for FMVO is ', num2str(median(rmse_phin))]);
disp(['Median RMSE of theta for FMVO is ', num2str(median(rmse_thetan))]);
disp(['Median RMSE of psi for FMVO is ', num2str(median(rmse_psin))]);



%% Position RMSE computation
rmse_x2 = 0; rmse_y2 = 0; rmse_z2 = 0;
rmse_xn = 0; rmse_yn = 0; rmse_zn = 0;
for i=1:N
    rmse_x2(i+1) = rmse_x2(i) + norm(grt_trI0(1,i) - new2(1,i))^2;
    rmse_y2(i+1) = rmse_y2(i) + norm(grt_trI0(2,i) - new2(2,i))^2;
    rmse_z2(i+1) = rmse_z2(i) + norm(grt_trI0(3,i) - new2(3,i))^2;

    rmse_xn(i+1) = rmse_xn(i) + norm(grt_trI0(1,i) - newn(1,i))^2;
    rmse_yn(i+1) = rmse_yn(i) + norm(grt_trI0(2,i) - newn(2,i))^2;
    rmse_zn(i+1) = rmse_zn(i) + norm(grt_trI0(3,i) - newn(3,i))^2;
end
rmse_x2 = sqrt(rmse_x2./(N+1));
rmse_y2 = sqrt(rmse_y2./(N+1));
rmse_z2 = sqrt(rmse_z2./(N+1));
rmse_xn = sqrt(rmse_xn./(N+1));
rmse_yn = sqrt(rmse_yn./(N+1));
rmse_zn = sqrt(rmse_zn./(N+1));

figure;
subplot(3, 1, 1)
plot(1:N+1,rmse_x2, 'b', 1:N+1, rmse_xn, 'r', 'LineWidth', 1.2);
ylabel('$m$', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend('RMSE($x_{MVO}$)', 'RMSE($x_{FMVO}$)', 'interpreter', 'latex', 'fontSize', 14);

subplot(3, 1, 2)
plot(1:N+1,rmse_y2, 'b', 1:N+1, rmse_yn, 'r', 'LineWidth', 1.2);
ylabel('$m$', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend('RMSE($y_{MVO}$)', 'RMSE($y_{FMVO}$)', 'interpreter', 'latex', 'fontSize', 14);

subplot(3, 1, 3)
plot(1:N+1,rmse_z2, 'b', 1:N+1, rmse_zn, 'r', 'LineWidth', 1.2);
xlabel('Sample Number', 'Interpreter', 'latex');
ylabel('$m$', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend('RMSE($z_{MVO}$)', 'RMSE($z_{FMVO}$)', 'interpreter', 'latex', 'fontSize', 14);



%% Time plot
accum_iter_time2 = 0;
accum_iter_timen = 0;
for i=1:N
    accum_iter_time2(i+1) = accum_iter_time2(i) + iter_time2(i);
    accum_iter_timen(i+1) = accum_iter_timen(i) + iter_timen(i);
end
figure;
plot(1:N,iter_time2, 'b', 1:N, iter_timen, 'r', 'LineWidth', 1.2);
xlabel('Sample Number', 'Interpreter', 'latex');
ylabel('Run-time ($s$)', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend({'MVO', 'FMVO'},'Location','northwest', 'interpreter', 'latex', 'FontSize', 14);
ylim([0 10]);

figure;
plot(1:N+1,accum_iter_time2, 'b', 1:N+1, accum_iter_timen, 'r', 'LineWidth', 1.2);
xlabel('Sample Number', 'Interpreter', 'latex');
ylabel('Accumulative run-time ($s$)', 'Interpreter', 'latex');
grid on;
box on;
set(gca,'FontSize', 14, 'TickLabelInterpreter', 'latex');
legend({'MVO', 'FMVO'},'Location','northwest', 'interpreter', 'latex', 'FontSize', 14);

%% Display

disp(['Median RMSE of x for MVO is ', num2str(median(rmse_x2))]);
disp(['Median RMSE of y for MVO is ', num2str(median(rmse_y2))]);
disp(['Median RMSE of z for MVO is ', num2str(median(rmse_z2))]);

disp(['Median RMSE of x for FMVO is ', num2str(median(rmse_xn))]);
disp(['Median RMSE of y for FMVO is ', num2str(median(rmse_yn))]);
disp(['Median RMSE of z for FMVO is ', num2str(median(rmse_zn))]);


disp(['Median run-time for each iteration for MVO is ', num2str(median(iter_time2))]);
disp(['Median run-time for each iteration for FMVO is ', num2str(median(iter_timen))]);




