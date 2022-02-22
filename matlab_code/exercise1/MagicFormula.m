function Y = MagicFormula(X,Bx,Cx,Dx,Ex,Sh,Sv)
x = X + Sh;
y = Dx*sin(Cx*atan(Bx*x-Ex*(Bx*x-atan(Bx*x))));
Y = y + Sv;