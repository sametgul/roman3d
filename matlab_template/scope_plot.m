figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot((error_scope.time-10), error_scope.signals.values);
ylim([-0.015 0.015])
xlabel("Time[sec]",'interpreter','latex','FontSize',14);
ylabel("$Errors$[m]",'interpreter','latex','FontSize',16);
yline(0.01,'--');
yline(-0.01,'--');
grid on;
legend('$e_1$','$e_2$','$\Delta$','interpreter','latex','NumColumns',3,...
    'FontSize',14,'location','southeast');
%saveas(gcf,'error_noConst','epsc')
%saveas(gcf,'error_5mm','epsc')
%saveas(gcf,'error_10mm','epsc')
%saveas(gcf,'error_20mm','epsc')
%saveas(gcf,'error_30mm','epsc')

figure('Renderer', 'painters', 'Position', [10 10 900 300]);
plot((torqueScope.time(10001:end)-10), torqueScope.signals.values((10001:end),:));
ylim([-3 3])
xlabel("Time[sec]",'interpreter','latex','FontSize',14);
ylabel("$\tau(t)$[volt]",'interpreter','latex','FontSize',14);
grid on;
legend('$\tau_1$','$\tau_2$','$\tau_3$',...
    'interpreter','latex','NumColumns',3,'FontSize',14,'location','southeast');
%saveas(gcf,'tau_noCnst','epsc')
%saveas(gcf,'tau_5mm','epsc')
%saveas(gcf,'tau_10mm','epsc')
%saveas(gcf,'tau_20mm','epsc')
%saveas(gcf,'tau_30mm','epsc')



figure('Renderer', 'painters');
plot(linksScope.time, linksScope.signals.values);
ylim([-40 120])
xlabel("Time[sec]",'interpreter','latex','FontSize',14);
ylabel("$q(t)$[deg]",'interpreter','latex','FontSize',14);
grid on;
legend('$q_1$','$q_2$','$q_3$',...
    'interpreter','latex','NumColumns',3,'FontSize',14);
%saveas(gcf,'links_noCnst','epsc')
%saveas(gcf,'links_9_36mm','epsc')
%saveas(gcf,'links_6_24mm','epsc')
%saveas(gcf,'links_3_12mm','epsc')


figure('Renderer', 'painters');
subplot 331
plot(estimationScope.time, estimationScope.signals.values(:,1));
ylabel("$\hat{\theta}_1$",'interpreter','latex','FontSize',14);
subplot 332
plot(estimationScope.time, estimationScope.signals.values(:,2));
ylabel("$\hat{\theta}_2$",'interpreter','latex','FontSize',14);
subplot 333
plot(estimationScope.time, estimationScope.signals.values(:,3));
ylabel("$\hat{\theta}_3$",'interpreter','latex','FontSize',14);
subplot 334
plot(estimationScope.time, estimationScope.signals.values(:,4));
ylabel("$\hat{\theta}_4$",'interpreter','latex','FontSize',14);
subplot 335
plot(estimationScope.time, estimationScope.signals.values(:,5));
ylabel("$\hat{\theta}_5$",'interpreter','latex','FontSize',14);
subplot 336
plot(estimationScope.time, estimationScope.signals.values(:,6));
ylabel("$\hat{\theta}_6$",'interpreter','latex','FontSize',14);
subplot 337
plot(estimationScope.time, estimationScope.signals.values(:,7));
ylabel("$\hat{\theta}_7$",'interpreter','latex','FontSize',14);
subplot 338
plot(estimationScope.time, estimationScope.signals.values(:,8));
ylabel("$\hat{\theta}_8$",'interpreter','latex','FontSize',14);
subplot 339
plot(estimationScope.time, estimationScope.signals.values(:,9));
ylabel("$\hat{\theta}_9$",'interpreter','latex','FontSize',14);
%saveas(gcf,'param_noCnst','epsc')
%saveas(gcf,'param_9_36mm','epsc')
%saveas(gcf,'param_6_24mm','epsc')
%saveas(gcf,'param_3_12mm','epsc')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Unconstraint Errors
ue1 = error_scope.signals.values(:,1);
ue2 = error_scope.signals.values(:,2);

%Constraint Errors
ce1 = error_scope.signals.values(:,1);
ce2 = error_scope.signals.values(:,2);

%Draw figure
figure('Renderer', 'painters');
plot(error_scope.time, ue2, 'b', error_scope.time, ce2, 'r');
ylim([-0.02 0.02])
xlabel("Time[sec]",'interpreter','latex','FontSize',14);
ylabel("$e_2(t)$[m]",'interpreter','latex','FontSize',14);
grid on;

yline(0.012,'--');
yline(-0.012,'--');

legend('$Unconstrained$','$Constrained$','$\Delta = 1.2\;[cm]$',...
    'interpreter','latex','NumColumns',1,'FontSize',14);
%legend('boxoff')
%saveas(gcf,'error2_compRed','epsc')

legend('$Unconstrained$','$Constrained$','$\Delta = 0.3\;[cm]$',...
    'interpreter','latex','NumColumns',1,'FontSize',14,...
    'location','southeast');

