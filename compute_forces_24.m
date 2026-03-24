function motor_forces = ComputeMotorForces(Fc, Gc, d, km)

X_c = Fc(1); % Likely to stay 0 the majority of the time
Y_c = Fc(2); % ^Ditto
Z_c = Fc(3);

L_c = Gc(1);
M_c = Gc(2);
N_c = Gc(3);


f_1 = -1 * (Z_c + L_c + M_c + N_c);
f_2 = (d / sqrt(2)) * ((-Z_c - L_c) + (M_c + N_c));
f_3 = (d / sqrt(2)) * ((-L_c - M_c) + (Z_c + N_c));
f_4 = (km) * ((-L_c - N_c) + (Z_c + M_c));



motor_forces = [f_1, f_2, f_3, f_4];

end