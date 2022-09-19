function StopSim()

global vrepInstance param jaco TCerebNet
vrepInstance.Disconnect(vrepInstance);

figure;
interval = 20;
plot3(jaco.desiredPath(1,1:end),jaco.desiredPath(2,1:end),jaco.desiredPath(3,1:end),'-', 'linewidth', 1);hold on;
plot3(jaco.actualPath(1,1:interval:end),jaco.actualPath(2,1:interval:end),jaco.actualPath(3,1:interval:end),'-.*', 'linewidth', 1);hold off
legend('Desired path','Actual path','fontsize',30,'fontname','times new roman');
set(gca,'FontSize',25);
grid on;
axis equal;
% title('Roll-outs: 1~3','fontname','times new roman');
xlabel("$x$ (m)",'interpreter','latex','fontsize',30); ylabel("$y$ (m)",'interpreter','latex','fontsize',30); zlabel("$z$ (m)",'interpreter','latex','fontsize',30);

figure
Te = 0:param.samplingGap:param.taskDuration;
p1 = plot(Te(1:interval:end),jaco.errors(1,1:interval:end)','-', 'linewidth', 3);
hold on;
p2 = plot(Te(1:interval:end),jaco.errors(2,1:interval:end)','--', 'linewidth', 3);%grid on;
p3 = plot(Te(1:interval:end),jaco.errors(3,1:interval:end)',':', 'linewidth', 3);hold off;
legend('$e_1$','$e_2$','$e_3$','fontsize',30,'interpreter','latex','NumColumns',3);
% ax = gca;
% ax.YAxis.Exponent = -3;
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('Position error (m)','fontname','times new roman','fontsize',30);

figure
p4 = plot(Te(1:interval:end),jaco.errors(4,1:interval:end)','-', 'linewidth', 3); hold on;
p5 = plot(Te(1:interval:end),jaco.errors(5,1:interval:end)',':', 'linewidth', 3);
p6 = plot(Te(1:interval:end),jaco.errors(6,1:interval:end)','--', 'linewidth', 3);
p7 = plot(Te(1:interval:end),jaco.errors(7,1:interval:end)','-.', 'linewidth', 3);hold off;
% title('Roll-outs: 1~3','fontname','times new roman');
legend('$e_4$','$e_5$','$e_6$','$e_7$','fontsize',30,'interpreter','latex','NumColumns',2);
% ax = gca;
% ax.YAxis.Exponent = -3;
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('Orientation error','fontname','times new roman','fontsize',30);

figure
p1 = plot(Te(1:interval:end),jaco.theta(1,1:interval:end)','-', 'linewidth', 3); hold on;
p2 = plot(Te(1:interval:end),jaco.theta(2,1:interval:end)',':', 'linewidth', 3);
p3 = plot(Te(1:interval:end),jaco.theta(3,1:interval:end)','--', 'linewidth', 3);
p4 = plot(Te(1:interval:end),jaco.theta(4,1:interval:end)','-.', 'linewidth', 3);
p3 = plot(Te(1:interval:end),jaco.theta(5,1:interval:end)','-', 'linewidth', 1);
p4 = plot(Te(1:interval:end),jaco.theta(6,1:interval:end)','-.', 'linewidth', 2);hold off;
% title('Roll-outs: 1~3','fontname','times new roman');
legend('$\theta_1$','$\theta_2$','$\theta_3$','$\theta_4$','$\theta_5$','$\theta_6$','fontsize',30,'interpreter','latex','NumColumns',3);
% ax = gca;
% ax.YAxis.Exponent = -3;
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('Joint angle (rad)','fontname','times new roman','fontsize',30);

% save (['Data/results_CIDGND.mat'],  'jaco','TCerebNet','param');

