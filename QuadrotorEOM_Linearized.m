function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
%% Extracting variables from state vector
    deltax = var(1);
    deltay = var(2);
    deltaz = var(3);
    deltaphi = var(4);
    deltatheta = var(5);
    deltapsi = var(6);
    deltau = var(7);
    deltav = var(8);
    deltaw = var(9);
    deltap = var(10);
    deltaq = var(11);
    deltar = var(12);

    %Inertias
    Ix = I(1,1);
    Iy = I(2,2);
    Iz = I(3,3);

    %Control pertubations
    deltaXc = deltaGc(1);
    deltaYc = deltaFc(2);
    deltaZc = deltaFc(3);
    deltaLc = deltaGc(1);
    deltaMc = deltaGc(2);
    deltaNc = deltaGc(3);

 %% Linearized Equations!

 deltaInertialVelocity = [deltau;deltav;deltaw];

 deltaEulerAngleRates = [deltap;deltaq;deltar];

 deltaBodyAccelerations = g.*[-deltatheta;deltaphi;0] + (1/m).*[deltaXc;deltaYc;deltaZc];

 deltaBodyAngleAccelerations = [(1/Ix).*deltaLc;(1/Iy).*deltaMc;(1/Iz).*deltaNc];

 var_dot = [deltaInertialVelocity; deltaEulerAngleRates; deltaBodyAccelerations; deltaBodyAngleAccelerations];

   


end

