function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)

% Task 2.3
% inputs: 12x1 state var, a/c mass m, gravity g
% outputs: control force vec Fc, control moment vector Gc

%force
Fc = [0;0;m*g];

%moments
gain = 0.004; %Nm/(rad/s)
Gc = -1 .* gain .* var(10:12);

end