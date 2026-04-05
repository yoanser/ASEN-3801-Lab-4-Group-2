function [Fc, Gc] = VelocityReferenceFeedback(t, var, vel_gains, lat_gains, long_gains, m, g)

if t <= 2
    ur = 0.5;
    vr = 0.5;
else
    ur = 0;
    vr = 0;
end

phi_r   =  vel_gains(1)*(vr - var(2));
theta_r = -vel_gains(2)*(ur - var(1));

Fc = [0;0;-m*g];

gain = 0.004;

deltaLc = lat_gains(1)*(phi_r - var(10)) - lat_gains(2)*var(4);
deltaMc = long_gains(1)*(theta_r - var(11)) - long_gains(2)*var(5);

Gc = [deltaLc; deltaMc; -gain*var(12)];

end