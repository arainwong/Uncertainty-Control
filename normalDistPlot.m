clear; clc; 
close all;

mu = 0;
sigma = 1;
normalDist = makedist('normal', 'mu', mu, 'sigma', sigma);

x = [-4:.1:4];
PDF = pdf(normalDist,x);
CDF = cdf(normalDist,x);


subplot(1,2,1);
plot(x, PDF, 'LineWidth', 1)
legend('PDF')
xlabel('x')
ylabel('f(x)')
% title('Normal Distribution')

subplot(1,2,2);
plot(x, CDF, 'LineWidth', 1)
legend('CDF')
xlabel('x')
ylabel('\Phi(x)')
% title('Normal Distribution')