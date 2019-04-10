%% Read in pose ground truth from Kitti odometry
% posesname = './dataset/poses/00.txt';
% T = readtable(posesname,'Delimiter','space','ReadRowNames',false,'ReadVariableNames',false);
% A = table2array(T);
% M = zeros(3,4,length(A));
% 
% for i = 1: length(A)
%     M(1:3,1:4,i) = [A(i,1:4);A(i,5:8);A(i,9:12)];
% end
% The folder 'poses' contains the ground truth poses (trajectory) for the
% first 11 sequences. This information can be used for training/tuning your
% method. Each file xx.txt contains a N x 12 table, where N is the number of
% frames of this sequence. Row i represents the i'th pose of the left camera
% coordinate system (i.e., z pointing forwards) via a 3x4 transformation
% matrix. The matrices are stored in row aligned order (the first entries
% correspond to the first row), and take a point in the i'th coordinate
% system and project it into the first (=0th) coordinate system. Hence, the
% translational part (3x1 vector of column 4) corresponds to the pose of the
% left camera coordinate system in the i'th frame with respect to the first
% (=0th) frame. Your submission results must be provided using the same data
% format.

%% Plot ground truth

clear all;
load('./data/M.mat')
load('./data/final_poses_SURF.mat')
load('./data/final_poses_SURF_inpos.mat')

%length(M)
len = length(M);
Po = zeros(3,len);
for j = 1 : len
    Po(:,j) = M(:,:,j)*[0;0;0;1];
end
pos = [0;0;0];
Rpos = eye(3);
for k = 1:len
    pos_temp = Po(:,k);
    Rpos_temp = M(:,1:3,k);
    tr(:,k) = inv(Rpos)*(pos_temp - pos);
    R(:,:,k) = inv(Rpos)*Rpos_temp;
    pos = pos_temp;
    Rpos = Rpos_temp;
end

locations = final_poses';
len =length(final_poses);
locationsGT = Po(:,1:len)';
magnitudes = sqrt(sum(locations.^2,2));
magnitudesGT = sqrt(sum(locationsGT.^2,2));
scaleFactor = median(magnitudesGT(2:end) ./ magnitudes(2:end));
new = final_poses.*scaleFactor;
figure;
%plot3(Po(1,1:len),Po(2,1:len),Po(3,1:len),-final_poses(1,1:len),final_poses(2,1:len),final_poses(3,1:len)...
   % ,-new(1,1:len),new(2,1:len),new(3,1:len))
%plot3(Po(1,1:len),Po(2,1:len),Po(3,1:len)...
%    ,-new(1,1:len),new(2,1:len),new(3,1:len))
% plot(Po(1,1:len),Po(3,1:len)...
%     ,-new(1,1:len),new(3,1:len),Po(1,40),Po(3,40),'*',Po(1, 110),Po(3,110),'*',...
%     Po(1,200),Po(3,200),'*',Po(1,430),Po(3,430),'*')

plot(Po(1,1:len),Po(3,1:len)...
    ,-new(1,1:len),new(3,1:len),'LineWidth',3)
xlabel('X (m)','FontSize',12)
ylabel('Z (m)','FontSize',12)
legend({'Ground Truth','Estimation with Scale'},'Location','SouthEast','FontSize',13)
title('Trajectory','FontSize',14)
%% Error Plot
error_x = Po(1,1:len)+new(1,1:len);
error_y = Po(2,1:len)-new(2,1:len);
error_z = Po(3,1:len)-new(3,1:len);
total_error =  abs(error_x)+abs(error_y)+abs(error_z);
figure;
t = 1:len;
plot(t,error_x,t,error_y,t,error_z,t,total_error)
legend('x error','y error','z error','total error','Location','SouthWest')
xlabel('Frames')
ylabel('Error (m)')
title('Error Plot')
%% Incremental Error
locations = result_pos';
len =length(result_pos);
locationsGT = tr(:,1:len)';
magnitudes = sqrt(sum(locations.^2,2));
magnitudesGT = sqrt(sum(locationsGT.^2,2));
scaleFactor = median(magnitudesGT(2:end) ./ magnitudes(2:end));
new = result_pos.*scaleFactor;
in_error_x = tr(1,2:len+1)-new(1,1:len);
in_error_y = tr(2,1:len)-new(2,1:len);
in_error_z = tr(3,1:len)-new(3,1:len);
in_total_error =  abs(in_error_x)+abs(in_error_y)+abs(in_error_z);
figure;
t = 1:len;
plot(t,in_error_x,t,in_error_y,t,in_error_z,t,in_total_error,40,in_total_error(40),'*',...
    110,in_total_error(110),'*',200,in_total_error(200),'*',430,in_total_error(430),'*')
legend('x error','y error','z error','total error','Location','SouthWest')
xlabel('Frames')
ylabel('Error (m)')
title('Incremental Error Plot')

%plot3(final_poses(1,1:537),final_poses(2,1:537),final_poses(3,1:537))
