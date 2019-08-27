%%

nr_tals =  csvread('/home/antequ/integratorplots/output_itlimiter_run1.csv', 1, 0);
nr_notals =  csvread('/home/antequ/integratorplots/output_itlimiter_runnotals.csv', 1, 0);

iterations_notals = nr_notals(:,2);
vt_notals = nr_notals(:,7);

iterations_tals = nr_tals(:,2);
vt_tals = nr_tals(:,7);
width = 230.4;
height = 115.2;
fig1 = figure('Units', 'points', 'Position', [1500 1000 width height]) % 3.2 " x 1.6 "
set(fig1, 'PaperUnits', 'points','PaperSize',[width+8 height+8])
p1 = plot(iterations_notals(1:20), vt_notals(1:20), '-ko', 'MarkerSize', 4)
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 8)
pbaspect([2 1 1])
ylim([-0.25,0.25])
xlabel('Newton-Raphson Iteration Count');
ylabel('\Delta{\itv_t} [m/s]');
saveas(fig1, '/home/antequ/integratorplots/NewtonRaphsonDivergence.pdf');
%print(fig1,'/home/antequ/integratorplots/NewtonRaphsonDivergence.pdf','-dpdf')
fig2 = figure('Units', 'points', 'Position', [1500 1000 width height]) % 3.2 " x 1.6 "
set(fig2, 'PaperUnits', 'points','PaperSize',[width+8 height+8])
p2 = semilogy(iterations_tals(1:20), abs(vt_tals(1:20)), '-ko', 'MarkerSize', 4)
set(gca, 'FontName', 'Times New Roman')
set(gca, 'FontSize', 8)
pbaspect([2 1 1])
xlabel('Newton-Raphson Iteration Count');
ylabel('|\Delta{\itv_t}| [m/s]');
saveas(fig2, '/home/antequ/integratorplots/NewtonRaphsonTalsConverges.pdf');

