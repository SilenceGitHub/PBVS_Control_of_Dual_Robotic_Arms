function StopSim()

global vrepInstance param kuka ur TCerebNet VCerebNet
vrepInstance.Disconnect(vrepInstance);

% % When loading existing data, please comment out the above two lines
% load Data/results_CIDGND.mat

interval = 2;

Te = 0:param.samplingGap:param.taskDuration;
stage1 = param.stage1Iter;
stage2 = param.stage2Iter;
if stage2 > round(param.taskDuration/param.samplingGap) + 1 || stage2 == 0
    stage2 = round(param.taskDuration/param.samplingGap) + 1;
end


figure
xMin = 0; xMax = stage2*param.samplingGap; yMin = -0.1; yMax = 0.2;
p1 = plot(Te(1:interval:stage2),ur.errors(1,1:interval:stage2)','-', 'linewidth', 4);
hold on;
p2 = plot(Te(1:interval:stage2),ur.errors(2,1:interval:stage2)','--', 'linewidth', 4);%grid on;
p3 = plot(Te(1:interval:stage2),ur.errors(3,1:interval:stage2)',':', 'linewidth', 4);
plot([stage1*param.samplingGap,stage1*param.samplingGap],[yMin,yMax],'g--','linewidth',3);
hold off;
legend('$e_{T,x}$','$e_{T,y}$','$e_{T,z}$','fontsize',30,'interpreter','latex','NumColumns',3);
axis([xMin xMax yMin yMax]);
set(gca, 'xtick', xMin:4:xMax);
set(gca, 'ytick', yMin:0.1:yMax);
set(gca,'FontSize',28);
xlabel('$t$ (s)','interpreter','latex','fontsize',32); ylabel('Position error (m)','fontname','times new roman','fontsize',32);

orien_d = zeros(4,stage2);
orien_a = zeros(4,stage2);
orien_d = angle2quat(ur.desiredPath(1,:), ur.desiredPath(2,:), ur.desiredPath(3,:),'XYZ')';
orien_a = angle2quat(ur.actualPath(1,:), ur.actualPath(2,:), ur.actualPath(3,:),'XYZ')';
orienError = orien_d - orien_a;

figure
xMin = 0; xMax = stage2*param.samplingGap; yMin = -0.05; yMax = 0.1;
p4 = plot(Te(1:interval:stage2),orienError(1,1:interval:stage2)','-', 'linewidth', 4); hold on;
p5 = plot(Te(1:interval:stage2),orienError(2,1:interval:stage2)',':', 'linewidth', 4);
p6 = plot(Te(1:interval:stage2),orienError(3,1:interval:stage2)','--', 'linewidth', 4);
p7 = plot(Te(1:interval:stage2),orienError(4,1:interval:stage2)','-.', 'linewidth', 4);
plot([stage1*param.samplingGap,stage1*param.samplingGap],[yMin,yMax],'g--','linewidth',4);hold off;
legend('$e_{T,w,1}$','$e_{T,w,2}$','$e_{T,w,3}$','$e_{T,w,4}$','fontsize',30,'interpreter','latex','NumColumns',2);
axis([xMin xMax yMin yMax]);
set(gca, 'xtick', xMin:4:xMax);
set(gca, 'ytick', yMin:0.05:yMax);
set(gca,'FontSize',28);
xlabel('$t$ (s)','interpreter','latex','fontsize',32); ylabel('Orientation error','fontname','times new roman','fontsize',32);


% save (['Data/results_CIDGND.mat'],  'ur','kuka','VCerebNet','TCerebNet','param');
end
