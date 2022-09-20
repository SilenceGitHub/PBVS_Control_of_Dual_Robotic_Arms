function StopSim()

global vrepInstance param kuka ur TCerebNet VCerebNet
vrepInstance.Disconnect(vrepInstance);

% load Data/results_CIDGND.mat

interval = 2;

Te = 0:param.samplingGap:param.taskDuration;
stage1 = param.stage1Iter;
stage2 = param.stage2Iter;

figure
xMin = 0; xMax = stage2*param.samplingGap; yMin = -0.1; yMax = 0.2;
p1 = plot(Te(1:interval:stage2),ur.errors(1,1:interval:stage2)','-', 'linewidth', 3);
hold on;
p2 = plot(Te(1:interval:stage2),ur.errors(2,1:interval:stage2)','--', 'linewidth', 3);%grid on;
p3 = plot(Te(1:interval:stage2),ur.errors(3,1:interval:stage2)',':', 'linewidth', 3);
plot([stage1*param.samplingGap,stage1*param.samplingGap],[yMin,yMax],'g--','linewidth',3);
hold off;
legend('$e_1$','$e_2$','$e_3$','fontsize',30,'interpreter','latex');
axis([xMin xMax yMin yMax]);
set(gca, 'xtick', xMin:4:xMax);
set(gca, 'ytick', yMin:0.1:yMax);
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('Position error (m)','fontname','times new roman','fontsize',30);


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
p6 = plot(Te(1:interval:stage2),orienError(4,1:interval:stage2)','-.', 'linewidth', 4);
plot([stage1*param.samplingGap,stage1*param.samplingGap],[yMin,yMax],'g--','linewidth',4);hold off;
legend('$e_4$','$e_5$','$e_6$','$e_7$','fontsize',32,'interpreter','latex','NumColumns',4);
axis([xMin xMax yMin yMax]);
set(gca, 'xtick', xMin:4:xMax);
set(gca, 'ytick', yMin:0.05:yMax);
set(gca,'FontSize',28);
xlabel('$t$ (s)','interpreter','latex','fontsize',32); ylabel('Orientation error','fontname','times new roman','fontsize',32);



figure
p1 = plot(Te(1:interval:stage2),kuka.errors(1,1:interval:stage2)','-', 'linewidth', 3);
hold on;
p2 = plot(Te(1:interval:stage2),kuka.errors(2,1:interval:stage2)','--', 'linewidth', 3);%grid on;
p3 = plot(Te(1:interval:stage2),kuka.errors(3,1:interval:stage2)',':', 'linewidth', 3);hold off;
legend('$e_x$','$e_y$','$e_z$','fontsize',30,'interpreter','latex');
xMin = 0; xMax = stage2*param.samplingGap; yMin = -0.1; yMax = 0.1;
axis([xMin xMax yMin yMax]);
set(gca, 'xtick', xMin:4:xMax);
set(gca, 'ytick', yMin:0.1:yMax);
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('$e(t)$ (m)','interpreter','latex','fontsize',30);

figure
p4 = plot(Te(1:interval:stage2),kuka.errors(4,1:interval:stage2)','-', 'linewidth', 3); hold on;
p5 = plot(Te(1:interval:stage2),kuka.errors(5,1:interval:stage2)',':', 'linewidth', 3);
p6 = plot(Te(1:interval:stage2),kuka.errors(6,1:interval:stage2)','--', 'linewidth', 3);
legend('$w_x$','$w_y$','$w_z$','fontsize',30,'interpreter','latex');
xMin = 0; xMax = stage2*param.samplingGap; yMin = -0.2; yMax = 0.3;
axis([xMin xMax yMin yMax]);
set(gca, 'xtick', xMin:4:xMax);
set(gca, 'ytick', yMin:0.1:yMax);
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('$e(t)$ (m)','interpreter','latex','fontsize',30);


% save (['Data/results_CIDGND.mat'],  'ur','kuka','VCerebNet','TCerebNet','param');

