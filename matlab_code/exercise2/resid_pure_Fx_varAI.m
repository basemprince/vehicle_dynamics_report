function res = resid_pure_Fx_varAI(X,FX,K,gamma,Fz,X_fz_nom)

    % ----------------------------------------------------------------------
    %% Compute the residuals - least squares approach - to fit the Fx curve 
    %  with Fz=Fz_nom, IA=0. Pacejka 1996 Magic Formula
    % ----------------------------------------------------------------------

    % Define MF coefficients
    
    pCx1 = X_fz_nom(1); 
    pDx1 = X_fz_nom(2);
    pEx1 = X_fz_nom(3);
    pEx4 = X_fz_nom(4);
    pKx1 = X_fz_nom(5);
    pHx1 = X_fz_nom(6);
    pVx1 = X_fz_nom(7);
    pDx2 = X_fz_nom(8);
    pEx2 = X_fz_nom(9);
    pEx3 = X_fz_nom(10);
    pHx2 = X_fz_nom(11);
    pKx2 = X_fz_nom(12);
    pKx3 = X_fz_nom(13);
    pVx2 = X_fz_nom(14);
    pDx3 = X(15);
    
    Fz0 = -890;
    dfz = Fz/Fz0 - 1;

    SHx = pHx1 + pHx2*dfz;
    Kx = K + SHx;
    Cx = pCx1;
    Ux = (pDx1 + pDx2*dfz) * (1 - pDx3 * gamma^2);
    Dx = Ux .* Fz;
    Kxk = Fz .* (pKx1 + pKx2*dfz) .* exp(-pKx3*dfz);
    Ex =(pEx1 + pEx2*dfz + pEx3*dfz.^2) .* (1 - pEx4*sign(Kx));
    Bx = Kxk / (Cx*Dx);
    SVx = Fz .* (pVx1 + pVx2*dfz);
    Fx0 = Dx .* sin(Cx * atan(Bx * Kx - Ex .* (Bx * Kx - atan(Bx * Kx)))) + SVx;  

    % Compute the residuals
    
    res = sum((FX - Fx0).^2)/sum(FX.^2);


end