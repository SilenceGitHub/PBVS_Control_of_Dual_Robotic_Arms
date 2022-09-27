function StopSim()

global vrepInstance param jaco TCerebNet
vrepInstance.Disconnect(vrepInstance);

% When loading existing data, please comment out the above two lines.
% load Data/results_CIDGND_Leaky9_LR1e-3_T1.mat


figure;
interval = 20;
plot3(jaco.desiredPath(1,1:end),jaco.desiredPath(2,1:end),jaco.desiredPath(3,1:end),'-', 'linewidth', 1);hold on;
plot3(jaco.actualPath(1,1:interval:end),jaco.actualPath(2,1:interval:end),jaco.actualPath(3,1:interval:end),'-.*', 'linewidth', 1);hold off
legend('Desired path','Actual path','fontsize',30,'fontname','times new roman','NumColumns',2);
set(gca,'FontSize',25);
grid on;
axis equal;
% title('Roll-outs: 1~3','fontname','times new roman');
xlabel("$x$ (m)",'interpreter','latex','fontsize',32); ylabel("$y$ (m)",'interpreter','latex','fontsize',32); zlabel("$z$ (m)",'interpreter','latex','fontsize',32);

figure
Te = 0:param.samplingGap:param.taskDuration;
p1 = plot(Te(1:interval:end),jaco.errors(1,1:interval:end)','-', 'linewidth', 4);
hold on;
p2 = plot(Te(1:interval:end),jaco.errors(2,1:interval:end)','--', 'linewidth', 4);%grid on;
p3 = plot(Te(1:interval:end),jaco.errors(3,1:interval:end)',':', 'linewidth', 4);hold off;
legend('$e_{T,x}$','$e_{T,y}$','$e_{T,z}$','fontsize',30,'interpreter','latex','NumColumns',3);
set(gca,'FontSize',28);
xlabel('$t$ (s)','interpreter','latex','fontsize',32); ylabel('Position error (m)','fontname','times new roman','fontsize',32);

RMSE = round(sqrt(sum(jaco.errors(1,:).^2 + jaco.errors(2,:).^2 + jaco.errors(3,:).^2)/2001), 8)

figure
p4 = plot(Te(1:interval:end),jaco.errors(4,1:interval:end)','-', 'linewidth', 4); hold on;
p5 = plot(Te(1:interval:end),jaco.errors(5,1:interval:end)',':', 'linewidth', 4);
p6 = plot(Te(1:interval:end),jaco.errors(6,1:interval:end)','--', 'linewidth', 4);
p7 = plot(Te(1:interval:end),jaco.errors(7,1:interval:end)','-.', 'linewidth', 4);hold off;
legend('$e_{T,w,1}$','$e_{T,w,2}$','$e_{T,w,3}$','$e_{T,w,4}$','fontsize',30,'interpreter','latex','NumColumns',2);
set(gca,'FontSize',28);
xlabel('$t$ (s)','interpreter','latex','fontsize',32); ylabel('Orientation error','fontname','times new roman','fontsize',32);

figure
p1 = plot(Te(1:interval:end),jaco.theta(1,1:interval:end)','-', 'linewidth', 3); hold on;
p2 = plot(Te(1:interval:end),jaco.theta(2,1:interval:end)',':', 'linewidth', 3);
p3 = plot(Te(1:interval:end),jaco.theta(3,1:interval:end)','--', 'linewidth', 3);
p4 = plot(Te(1:interval:end),jaco.theta(4,1:interval:end)','-.', 'linewidth', 3);
p3 = plot(Te(1:interval:end),jaco.theta(5,1:interval:end)','-', 'linewidth', 1);
p4 = plot(Te(1:interval:end),jaco.theta(6,1:interval:end)','-.', 'linewidth', 2);hold off;
legend('$\theta_{T,1}$','$\theta_{T,2}$','$\theta_{T,3}$','$\theta_{T,4}$','$\theta_{T,5}$','$\theta_{T,6}$','fontsize',30,'interpreter','latex','NumColumns',3);
set(gca,'FontSize',25);
xlabel('$t$ (s)','interpreter','latex','fontsize',30); ylabel('Joint angle (rad)','fontname','times new roman','fontsize',30);


% save (['Data/results_CIDGND_Leaky9_LR1e-3_T1.mat'],  'jaco','TCerebNet','param');

