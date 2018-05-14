function dq = EdumipKinematics( t, q, v ,omega )
% Edumip kinematics
dq = [ v*cos(q(3)); v*sin(q(3)); omega ];
