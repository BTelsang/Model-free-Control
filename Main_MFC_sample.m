clc;
clear;

N = 200;
tsamp = 15; % Sampling time
L = 6*tsamp; % L is a parameter required in MFC 
endtime = tsamp*N;
timevector = 0:tsamp:endtime-tsamp;

parameters.L = L;
parameters.alpha = 1; % another parameter in MFC. This alpha should be 
% such that it magnitude of y and alpha*u should be same.
parameters.tsamp = tsamp;
parameters.Kp = 3.2; % Kp in the P controller. Tune it a little bit 
% if necessary based on your model


% This part is generating data from a sample model to be used 
% in MFC. 
sys.A =  0.9998;
sys.B =  0.2565;
sys.C = 1;
sys.D = 0;
u = ones(1,length(timevector));
y = zeros(1,length(timevector));
x(:,length(timevector)) = zeros(length(sys.A),1);
x(:,1) = 0; 
for k = 1:length(timevector)
    y(k) = sys.C * x(:,k) + sys.D * u(k);
    x(:,k+1) = sys.A * x(:,k) + sys.B * u(k);
end

% Control part prelim: This is reference trajectory generation. Whatever
% desired output you want, make that the setpoint. 
setpoint = 3*ones(1,length(timevector));
G_inertialComp = tf(1,[1 1]);
ref = lsim(G_inertialComp,setpoint,timevector)';
G_inertialComp_ddt = tf([1 0],[1 1]);
dref = lsim(G_inertialComp_ddt,setpoint,timevector)';

% Initial settings 
n = length(0:tsamp:L); %n is the number of samples in [t-L,t]
a = 0;
k_a = find(abs(timevector-(a)) < 1e-6); 
b = a + L;
k_b = find(abs(timevector-(b)) < 1e-6); 

for i = 1:length(timevector)-n
    reference.ref = ref(k_b);
    reference.dref = dref(k_b);
    
    e(i) = y(k_b) - ref(k_b);  
    
    % This is the core MFC. Give it previous few input and output 
    % measurements, reference and the parameters and you will get the
    % desired control signal uc.
    uc = MFC(u(k_a:k_b),y(k_a:k_b),reference,parameters);
    
    u(k_b+1) = uc;
    y(k_b+1) = sys.C*x(:,k_b+1) + sys.D*u(k_b+1);
    x(:,k_b+2) = sys.A*x(:,k_b+1) + sys.B*u(k_b+1);
    
    a = a + tsamp;
    b = b + tsamp;
    k_a = find(abs(timevector-(a)) < 1e-6); 
    k_b = find(abs(timevector-(b)) < 1e-6); 
end

% Plots for analysis
figure;
timevector = 0:1:length(setpoint)-1;
plot(timevector,setpoint,'-.k',timevector,ref,timevector,y,'LineWidth',2);
legend({'Setpoint','Reference','Measured'},'Location','Southwest','FontSize',12)
title(['# of samples = ' num2str(n)],'FontSize', 15)
xlabel('Time in seconds','FontSize', 15)
ylabel('Output of the system','FontSize', 15)

figure; plot(e); title('Error')
