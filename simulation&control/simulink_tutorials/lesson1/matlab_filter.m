t = linspace(0,2*pi,100);
rng default  %initialize random number generator
x = sin(t) + 0.25*rand(size(t));
windowSize = 5; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y = filter(b,a,x);
plot(t,x,t,y);
legend('Input Data','Filtered Data');