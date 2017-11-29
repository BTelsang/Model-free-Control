function uc = MFC(u,y,reference,parameters)

% Inputs: u,y, last ref, last dref, parameters(contains tsamp, alpha and L)
% Output: control signal uc

alpha = parameters.alpha;
L = parameters.L;
tsamp = parameters.tsamp;
Kp = parameters.Kp;

ref = reference.ref;
dref = reference.dref;

sigma = 0:tsamp:L;
y_sigma = y;
u_sigma = u;

term_y = (L-2*sigma).*y_sigma;
term_u = alpha.*sigma.*(L-sigma).*u_sigma;
to_be_integrated = -6*(term_y + term_u)/(L^3);
Phi = trapz(to_be_integrated)*tsamp;

e = y(end) - ref;
uc = Phi - dref + Kp*e;
uc = -uc/alpha;
