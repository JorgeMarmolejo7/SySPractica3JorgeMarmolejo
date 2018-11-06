f= @(omega) a./(a.^2+omega.^2);
a=1;
t=[-2:2];
f(t);
plot(t, f(t));
xlabel('t'); ylabel('f(t)=a/(a^2+\omega ^2)'); grid;