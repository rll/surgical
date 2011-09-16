D = load('~/rll/surgical/DiscreteRods/environmentFiles/w2/w2_data_plot.txt')

h=figure
hold on

p_open = plot(D(1:7,2),D(1:7,3))
set(p_open, 'Color', [.2,.8,.2], 'LineWidth', 1)

p_close5 = plot(D(8:14,2),D(8:14,3))
set(p_close5, 'Color', [.2,.2,.8], 'LineWidth', 1)

p_close10 = plot(D(15:21,2),D(15:21,3))
set(p_close10, 'Color', [.8,.2,.2], 'LineWidth', 1)

hleg = legend('Open Loop', 'Closed Loop, Horizon 5', 'Closed Loop, Horizon 10')
set(hleg,'FontSize', 10,'TextColor',[0,0,0])

ylabel('Success Rate')
xlabel('Noise')

set(findobj('type','axis'), 'FontSize', 16)

print(h, '-dpdf', '~/rll/surgical/DiscreteRods/environmentFiles/w2/w2_open_closed_loop_success_noise.pdf', '-r125')