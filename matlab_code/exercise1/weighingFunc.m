function Y = weighingFunc(Dxa, Cxa, Bxa, alpha, SHxa)
x = alpha + SHxa;
y = atan(Bxa*x);
Y = Dxa .* cos(Cxa*y);
