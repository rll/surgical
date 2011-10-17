prob = 'w4'

D = load(strcat('~/rll/surgical/DiscreteRods/environmentFiles/',prob,'/',prob,'_data_plot.txt'))
Ds = load(strcat('~/rll/surgical/DiscreteRods/environmentFiles/',prob,'s/',prob,'s_data_plot.txt'))

s = size(D,1)

h0 = 1:size(D,1)/2;
h5 = size(D,1)/2+1:size(D,1);


h=figure
hold on




p_open = plot(D(h0,2),D(h0,3))
set(p_open, 'Color', [.2,.8,.2], 'LineWidth', 1)

p_close5 = plot(D(h5,2),D(h5,3))
set(p_close5, 'Color', [.2,.2,.8], 'LineWidth', 1)

p_opens = plot(Ds(h0,2),Ds(h0,3))
set(p_opens, 'Color', [.8,.2,.2], 'LineWidth', 1)

p_close5s = plot(Ds(h5,2),Ds(h5,3))
set(p_close5s, 'Color', [.2,.8,.8], 'LineWidth', 1)

hleg = legend('Open Loop', 'Closed Loop', 'Smoothed Open Loop', 'Smoothed Closed Loop')
set(hleg,'FontSize', 10,'TextColor',[0,0,0], 'Location', 'SouthEast')

ylabel('Success Rate')
xlabel('\nu')

set(findobj('type','axis'), 'FontSize', 24)

print(h, '-dpdf', strcat('~/rll/surgical/DiscreteRods/environmentFiles/',prob,'/',prob,'_open_closed_loop_success_noise.pdf'), '-r125')