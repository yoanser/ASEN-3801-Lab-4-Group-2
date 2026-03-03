clear; clc;

function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)

x = var(1);
y = var(2);
z = var(3);
phi = var(4);
theta = var(5);
psi = var(6);
u = var(7);
v = var(8);
w = var(9);
p = var(10);
q = var(11);
r = var(12);

s_ph = sin(phi);
s_th = sin(theta);
s_ps = sin(psi);
c_ph = cos(phi);
c_th = cos(theta);
c_ps = cos(psi);


x_dot = (c_th * c_ps * u) + ((s_ph * s_th * c_ps - c_ph * s_ps * v) + ((c_ph * s_th * c_ps) + s_ph * s_ps) * w);
y_dot = (c_th * s_ps * u) + (((s_ph * s_th * s_ps) + (c_phi * c_ps)) * v) + (((c_ph * s_th * s_ps) - (s_ph * c_psi)) * w);
z_dot = (-s_th * u) + (s_ph * c_th * v) + c_ph * c_th * w;
phi_dot = p + (s_ph * tan(theta)
theta_dot =
psi_dot =
u_dot =
v_dot =
w_dot =
p_dot =
q_dot =
r_dot =





var_dot = [x_dot , y_dot , z_dot , phi_dot , theta_dot , psi_dot , u_dot , v_dot , w_dot , p_dot , q_dot , r_dot];
end




