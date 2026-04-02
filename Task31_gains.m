function [lat_gains, long_gains] = Task31_gains (I)

I_x = I(1,1);
I_y = I(2,2);

%time constants
tau1 = 0.5;
tau2 = 0.05;

%sigma
sigma1 = -1/tau1;
sigma2 = -1/tau2;

%set them as eigenvalues (aka roots)
roots = [sigma1 sigma2];
coeffs = poly(roots);

%lateral
k1_lat = coeffs(2)*I_x; %derivative gain
k2_lat = coeffs(3)*I_x; %proportional gain
lat_gains = [k1_lat k2_lat];

%longitudinal
k1_long = coeffs(2)*I_y;
k2_long = coeffs(3)*I_y;
long_gains = [k1_long k2_long];



end
