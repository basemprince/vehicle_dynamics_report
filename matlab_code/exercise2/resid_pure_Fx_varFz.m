function res = resid_pure_Fx_varFz(X,FX,K,gamma,Fz,X_fz_nom)

    % ----------------------------------------------------------------------
    %% Compute the residuals - least squares approach - to fit the Fx curve 
    %  with Fz=Fz_nom, IA=0. Pacejka 1996 Magic Formula
    % ----------------------------------------------------------------------

    % Define MF coefficients

%     Fz0 = ...
%     
%     pCx1 = X(1); 
%     pDx1 = X(2);
      
%     ...
%      
%     Dx = ...
%     Bx = ...    
%     Fx0 = Dx*...
    
    pCx1 = X_fz_nom(1); 
    pDx1 = X_fz_nom(2);
    pEx1 = X_fz_nom(3);
    pEx4 = X_fz_nom(4);
    pKx1 = X_fz_nom(5);
    pHx1 = X_fz_nom(6);
    pVx1 = X_fz_nom(7);
    pDx2 = X(8);
    pEx2 = X(9);
    pEx3 = X(10);
    pHx2 = X(11);
    pKx2 = X(12);
    pKx3 = X(13);
    pVx2 = X(14);
    pDx3 = X(15);
    
    Fz0 = 890;
    dfz = (Fz/Fz0) - 1;
%     disp(size(dfz));
    SHx = pHx1 + (pHx2*dfz);
%     disp(size(SHx));
    Kx = K + SHx;
%     disp(size(Kx));
    Cx = pCx1;
    mux = (pDx1+(pDx2*dfz))*(1-(pDx3*gamma^2));
%     disp(size(mux));
    Dx = mux.*Fz;
%     disp(size(Dx));
    Kxk = Fz.*(pKx1+(pKx2*dfz)).* exp(-pKx3*dfz);
%     disp(size(Kxk));
    Ex = (pEx1+(pEx2*dfz) + (pEx3*dfz.^2)) .* (1-(pEx4*sign(Kx)));
%     disp(size(Ex));
    Bx = Kxk / (Cx*Dx);
    SVx = Fz.*(pVx1+(pVx2*dfz));
    a = atan(Bx*Kx);
    b = (Bx*Kx)-a;
    c = (Bx*Kx)-(Ex .*b);
    d = sin(Cx * atan(c));
    Fx0 = (Dx .* d) + SVx;
    
%     disp(size(Bx));
%     disp(size(b));
%     disp(size(c));
%     disp(size(d));
%     disp(size(Fx0));
%     disp(Fx0);
    
    % Compute the residuals
    
    res = sum((FX - Fx0).^2)/sum(FX.^2);

end

