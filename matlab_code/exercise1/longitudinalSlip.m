function X = longitudinalSlip(w,Re,Vcx)
x1 = w * Re;
x2 = x1 - Vcx;
X = x2/Vcx;