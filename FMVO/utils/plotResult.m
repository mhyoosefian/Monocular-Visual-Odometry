function rms_error = plotResult(finalRot, grt_data, finalTr)
%% Orientation
% the rotation between cam and imu frames (from imu to cam)
RI0C0 = [0.014865542981800  -0.999880929698000   0.004140296794220
    0.999557249008000   0.014967213324700   0.025715529948000
    -0.025774436697400   0.003756188357970   0.999660727178000];
N = length(finalTr);
eul = zeros(N, 3);
for i=1:N
     RI0(:,:,i) = RI0C0*finalRot(:,:,i).'*inv(RI0C0);
     eul(i,:) = rotm2eul(RI0(:,:,i));
end
phi = eul(:,3);
theta = eul(:,2);
psi = eul(:,1);

grt_quat = grt_data(:,5:8);
grt_quat = grt_quat(1:N,:);
RWI0 = quat2rotm(grt_quat(1,:));

for i=1:N
    grt_RI0(:,:,i) = inv(RWI0)*quat2rotm(grt_quat(i,:)); 
%     RI0(:,:,i) = inv(RI0(:,:,1))*RI0(:,:,i); 
end
eul_true = rotm2eul(grt_RI0);


figure;
plot(1:N,phi*180/pi, 'b--', 1:N, eul_true(:,3)*180/pi, 'b');
hold on
plot(1:N,theta*180/pi, 'g--', 1:N, eul_true(:,2)*180/pi, 'g');
plot(1:N,psi*180/pi, 'r--', 1:N, eul_true(:,1)*180/pi, 'r');
legend('\phi', '\phi_{true}', '\theta', '\theta_{true}', '\psi', '\psi_{true}');
xlabel('Sample Number'); ylabel('Degrees');


rmse_phi = 0; rmse_theta = 0; rmse_psi = 0;
for i=1:N
    rmse_phi(i) = norm(phi(i) - eul_true(i,3))/N;
    rmse_theta(i) = norm(theta(i) - eul_true(i,2))/N;
    rmse_psi(i) = norm(psi(i) - eul_true(i,1))/N;
end
rmse_phi = sqrt(rmse_phi); rmse_theta = sqrt(rmse_theta); rmse_psi = sqrt(rmse_psi);
rmse_orientation = [rmse_phi; rmse_theta ; rmse_psi];

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
    trI0(:,i) = trI0C0 + RI0C0*finalTr(:,i) - RI0(:,:,i)*trI0C0;
end
magnitudes = sqrt(sum((trI0').^2,2));
magnitudesGT = sqrt(sum((grt_trI0').^2,2));
scaleFactor = median(magnitudesGT(2:end) ./ magnitudes(2:end));
new = trI0.*scaleFactor;
figure;
plot(grt_trI0(1,1:N),grt_trI0(3,1:N)...
    ,-new(1,1:N),-new(3,1:N))
xlabel('X (m)')
ylabel('Z (m)')
grid on
legend({'Ground Truth','Estimation with Scale'},'Location','SouthEast')
title('Trajectory')

figure;
plot(grt_trI0(1,1:N),grt_trI0(2,1:N)...
    ,-new(1,1:N),-new(2,1:N))
xlabel('X (m)')
ylabel('Y (m)')
grid on
legend({'Ground Truth','Estimation with Scale'},'Location','SouthEast')
title('Trajectory')

figure;
plot3(grt_trI0(1,1:N),grt_trI0(2,1:N),grt_trI0(2,1:N));
hold on;
plot3(-new(1,1:N),-new(2,1:N),-new(3,1:N));
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
grid on
box on
legend('Ground Truth','Estimation with Scale')
title('Trajectory')

rmse_x = 0; rmse_y = 0; rmse_z = 0;
for i=1:N
    rmse_x(i) = norm(grt_trI0(1,i) - new(1,i))/N;
    rmse_y(i) = norm(grt_trI0(2,i) - new(2,i))/N;
    rmse_z(i) = norm(grt_trI0(3,i) - new(3,i))/N;
end
rmse_x = sqrt(rmse_x); rmse_y = sqrt(rmse_y); rmse_z = sqrt(rmse_z);
rmse_position = [rmse_x; rmse_y ; rmse_z];

rms_error = [rmse_orientation; rmse_position];
end