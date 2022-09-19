function StopSim()

global vrepInstance param kuka VCerebNet
vrepInstance.Disconnect(vrepInstance);

interval = 1;

figure
Te = 0:param.samplingGap:param.taskDuration;
p1 = plot(Te(1:interval:end),kuka.errors(1,1:interval:end)','-', 'linewidth', 3);
hold on;
p2 = plot(Te(1:interval:end),kuka.errors(2,1:interval:end)','--', 'linewidth', 3);%grid on;
p3 = plot(Te(1:interval:end),kuka.errors(3,1:interval:end)',':', 'linewidth', 3);hold off;
legend('$e_1$','$e_2$','$e_3$','fontsize',30,'interpreter','latex','NumColumns',3);
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('Position error (m)','fontname','times new roman','fontsize',30);

figure
p4 = plot(Te(1:interval:end),-kuka.errors(4,1:interval:end)','-', 'linewidth', 3); hold on;
p5 = plot(Te(1:interval:end),-kuka.errors(5,1:interval:end)',':', 'linewidth', 3);
p6 = plot(Te(1:interval:end),-kuka.errors(6,1:interval:end)','--', 'linewidth', 3);
p7 = plot(Te(1:interval:end),-kuka.errors(7,1:interval:end)','-.', 'linewidth', 3);hold off;
legend('$e_4$','$e_5$','$e_6$','$e_7$','fontsize',30,'interpreter','latex','NumColumns',2);
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('Orientation error','fontname','times new roman','fontsize',30);

figure
p1 = plot(Te(1:interval:end),kuka.theta(1,1:interval:end)','-', 'linewidth', 3); hold on;
p2 = plot(Te(1:interval:end),kuka.theta(2,1:interval:end)',':', 'linewidth', 3);
p3 = plot(Te(1:interval:end),kuka.theta(3,1:interval:end)','--', 'linewidth', 3);
p4 = plot(Te(1:interval:end),kuka.theta(4,1:interval:end)','-.', 'linewidth', 3);
p3 = plot(Te(1:interval:end),kuka.theta(5,1:interval:end)','-', 'linewidth', 1);
p4 = plot(Te(1:interval:end),kuka.theta(6,1:interval:end)','-.', 'linewidth', 2);hold off;
legend('$\theta_1$','$\theta_2$','$\theta_3$','$\theta_4$','$\theta_5$','$\theta_6$','fontsize',30,'interpreter','latex','NumColumns',3);
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('Joint angle (rad)','fontname','times new roman','fontsize',30);

save (['Data/results_CIDGND.mat'],  'kuka','VCerebNet','param');

