function StopSim()

global vrepInstance param kuka VCerebNet
% vrepInstance.Disconnect(vrepInstance);

% % When loading existing data, please comment out the above two lines
% load Data/results_CIDGND.mat

interval = 1;

figure
Te = 0:param.samplingGap:param.taskDuration;
p1 = plot(Te(1:interval:end),kuka.errors(1,1:interval:end)','-', 'linewidth', 4);
hold on;
p2 = plot(Te(1:interval:end),kuka.errors(2,1:interval:end)','--', 'linewidth', 4);%grid on;
p3 = plot(Te(1:interval:end),kuka.errors(3,1:interval:end)',':', 'linewidth', 4);hold off;
legend('$e_{V,x}$','$e_{V,y}$','$e_{V,z}$','fontsize',30,'interpreter','latex','NumColumns',3);
set(gca,'FontSize',28);
xlabel('$t$ (s)','interpreter','latex','fontsize',32); ylabel('Position error (m)','fontname','times new roman','fontsize',32);

figure
p4 = plot(Te(1:interval:end),-kuka.errors(4,1:interval:end)','-', 'linewidth', 4); hold on;
p5 = plot(Te(1:interval:end),-kuka.errors(5,1:interval:end)',':', 'linewidth', 4);
p6 = plot(Te(1:interval:end),-kuka.errors(6,1:interval:end)','--', 'linewidth', 4);
p7 = plot(Te(1:interval:end),-kuka.errors(7,1:interval:end)','-.', 'linewidth', 4);hold off;
legend('$e_{V,w,1}$','$e_{V,w,2}$','$e_{V,w,3}$','$e_{V,w,4}$','fontsize',30,'interpreter','latex','NumColumns',2);
set(gca,'FontSize',28);
xlabel('$t$ (s)','interpreter','latex','fontsize',32); ylabel('Orientation error','fontname','times new roman','fontsize',32);

figure
p1 = plot(Te(1:interval:end),kuka.theta(1,1:interval:end)','-', 'linewidth', 3); hold on;
p2 = plot(Te(1:interval:end),kuka.theta(2,1:interval:end)',':', 'linewidth', 3);
p3 = plot(Te(1:interval:end),kuka.theta(3,1:interval:end)','--', 'linewidth', 3);
p4 = plot(Te(1:interval:end),kuka.theta(4,1:interval:end)','-.', 'linewidth', 3);
p5 = plot(Te(1:interval:end),kuka.theta(5,1:interval:end)','-', 'linewidth', 1);
p6 = plot(Te(1:interval:end),kuka.theta(6,1:interval:end)','-.', 'linewidth', 2);hold off;
legend('$\theta_{V,1}$','$\theta_{V,2}$','$\theta_{V,3}$','$\theta_{V,4}$','$\theta_{V,5}$','$\theta_{V,6}$','fontsize',30,'interpreter','latex','NumColumns',3);
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('Joint angle (rad)','fontname','times new roman','fontsize',30);

% save (['Data/results_CIDGND.mat'],  'kuka','VCerebNet','param');

