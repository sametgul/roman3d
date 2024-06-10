%Tau1 L2 calculations

tau1_10to70 = torqueScope.signals.values(10001:70000,1);
tau1_10to70_Square   = tau1_10to70.*tau1_10to70;

sum_tau1_10to70_Square = 0;

for c = 1:length(tau1_10to70_Square)
   sum_tau1_10to70_Square = sum_tau1_10to70_Square+tau1_10to70_Square(c);
end

L2_stable_tau1 = sqrt(sum_tau1_10to70_Square)


%Tau2 L2 calculations

tau2_10to70 = torqueScope.signals.values(10001:70000,2);
tau2_10to70_Square   = tau2_10to70.*tau2_10to70;

sum_tau2_10to70_Square = 0;

for c = 1:length(tau2_10to70_Square)
   sum_tau2_10to70_Square = sum_tau2_10to70_Square+tau2_10to70_Square(c);
end

L2_stable_tau2 = sqrt(sum_tau2_10to70_Square)

%Tau3 L2 calculations

tau3_10to70 = torqueScope.signals.values(10001:70000,3);
tau3_10to70_Square   = tau3_10to70.*tau3_10to70;

sum_tau3_10to70_Square = 0;

for c = 1:length(tau3_10to70_Square)
   sum_tau3_10to70_Square = sum_tau3_10to70_Square+tau3_10to70_Square(c);
end

L2_stable_tau3 = sqrt(sum_tau3_10to70_Square)