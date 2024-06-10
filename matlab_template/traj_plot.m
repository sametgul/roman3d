%font size
fsize = 16;

%desired and actual trajectories figure
figure('Renderer', 'painters');
plot(xdes_scope.signals.values(:,1),... 
    xdes_scope.signals.values(:,2),'--r',...
    x_scope.signals.values(:,1),...
    x_scope.signals.values(:,2),'b');
title('Desired vs Actual');
legend("Desired",'Actual','interpreter','latex','FontSize',(fsize-2));
xlabel("$x_1\;vs\;x_{d1}$",'interpreter','latex','FontSize',fsize);
ylabel("$x_2\;vs\;x_{d2}$",'interpreter','latex','FontSize',fsize);
%ylim([0.22 0.38]);
%xlim([0.05 0.36]);
grid on;

% %desired trajectory figure
% figure('Renderer', 'painters');
% plot(xdes_scope.signals.values(:,1),... 
%     xdes_scope.signals.values(:,2),'--r');
% title('Desired Trajectory');
% xlabel("$x_{d1}$",'interpreter','latex','FontSize',fsize);
% ylabel("$x_{d2}$",'interpreter','latex','FontSize',fsize);
% ylim([0.2 0.6]);
% xlim([0 0.6]);
% grid on;
% 
% %actual trajectory figure
% figure('Renderer', 'painters');
% plot(x_scope.signals.values(:,1),...
%     x_scope.signals.values(:,2),'b');
% title('Actual Trajectory');
% xlabel("$x_{1}$",'interpreter','latex','FontSize',fsize);
% ylabel("$x_{2}$",'interpreter','latex','FontSize',fsize);
% ylim([0.2 0.6]);
% xlim([0 0.6]);
% grid on;

all = mean(estimate_scope.signals.values(20001:end,:))
fd1_mean = mean(estimate_scope.signals.values(20001:end,7))
fd2_mean = mean(estimate_scope.signals.values(20001:end,8))
fd3_mean = mean(estimate_scope.signals.values(20001:end,9))