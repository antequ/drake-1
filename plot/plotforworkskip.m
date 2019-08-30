

tamsi = csvread('/home/antequ/integratorplots/impstribeck3ms.csv', 1, 0);
tals = csvread('/home/antequ/integratorplots/point_contact_benchmark_itlimiter5ms.csv', 1, 0);
notals = csvread('/home/antequ/integratorplots/point_contact_benchmark_noitlimiter5ms.csv', 1, 0);
comp_effort_notals = notals(:,2);
sim_time_notals = notals(:,1);
comp_effort_tals = tals(:,2);
sim_time_tals = tals(:,1);


% make similar for box

width = 230.4;
height = 230.4/3;
% for video
fig1 = figure('Units', 'points', 'Position', [1000 1000 width height],'PaperUnits', 'points','PaperSize',[width+8 height+8]);
%set(gcf,'Color','k');
p=plot(comp_effort_tals/1e6, sim_time_tals, 'g',...
   comp_effort_notals/1e6, sim_time_notals, 'r');
%set(gca,'Color','k');
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 8)
axis tight
xlabel("Wall Clock Time (s)")
ylabel("Sim Time (s)")
%set(gca, 'xcolor', 'w')
%set(gca, 'ycolor', 'w')

lgd=legend("TALS (ours)", "Implicit Euler");
lgd.Location = 'southeast';
legend boxoff 
set(lgd,'color','k');
saveas(fig1, '/home/antequ/integratorplots/SimpleGripperRuntimesVideo.pdf')

tals = csvread('/home/antequ/integratorplots/point_contact_benchmark_itlimiter3ms.csv', 1, 0);
notals = csvread('/home/antequ/integratorplots/point_contact_benchmark_noitlimiter3ms.csv', 1, 0);
comp_effort_notals = notals(:,2);
sim_time_notals = notals(:,1);
comp_effort_tals = tals(:,2);
sim_time_tals = tals(:,1);

% for paper
width = 230.4;
height = 230.4/2;
fig1 = figure('Units', 'points', 'Position', [1000 1000 width height],'PaperUnits', 'points','PaperSize',[width+8 height+8]);
%set(gcf,'Color','k');
p=plot(comp_effort_tamsi/1e6, sim_time_tamsi, 'k',...
    comp_effort_tals/1e6, sim_time_tals, '--k',...
   comp_effort_notals/1e6, sim_time_notals, 'k');
p(1).LineWidth=1.2;
%set(gca,'Color','k');
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 8)
axis tight
xlabel("CPU Wall Clock Time [s]")
ylabel("Simulation Time [s]")
%set(gca, 'xcolor', 'w')
%set(gca, 'ycolor', 'w')

lgd=legend("TAMSI","IE+FN+TALS", "IE+FN");
lgd.Location = 'southeast';
legend boxoff 
set(lgd,'color','k');
saveas(fig1, '/home/antequ/integratorplots/SimpleGripperRuntimesPaper.pdf')
