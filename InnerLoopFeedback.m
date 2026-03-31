function [Fc, Gc] = InnerLoopFeedback(var,lat_gains,long_gains)

%force
Fc = [0;0;-m*g];

%moments
gain = 0.004; %Nm/(rad/s)
deltaLc = -lat_gains(1)*var(10) -lat_gains(2)*var(4);
deltaMc = -long_gains(1)*var(11) -long_gains(2)*var(5);
Gc = [deltaLc;deltaMc;-gain*var(12)];

end