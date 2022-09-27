clear all
close all

load Data/results_DGND.mat
errors = jaco.errors;
Euclidean = sqrt(errors(1,:).^2 + errors(2,:).^2 + errors(3,:).^2);
box1 =Euclidean';

numTrial = 10;
for trial = 1:numTrial
    load (['Data/results_CIDGND_T',num2str(trial),'.mat'])
    errors = jaco.errors;
    Euclidean = sqrt(errors(1,:).^2 + errors(2,:).^2 + errors(3,:).^2);
    box1 =[box1, Euclidean'];
end

figure;
interval = 1;
boxplot(box1(:,1:interval:end),1:numTrial+1);grid on;
set(gca,'FontSize',24);
set(gca,'xticklabel',{'T0','T1','T2','T3','T4','T5','T6','T7','T8','T9','T10'},'fontname','times new roman'); 
% xlabel("Test",'fontsize',18,'fontname','times new roman');
ylabel("Error (m)",'fontsize',26,'fontname','times new roman');
% title('Robot 1 tracking error');
