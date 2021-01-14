A = [-0.0239 -0.0494 0 -32.2;
    -0.0145 -0.577 347 0;
    -8.396*10^-7 -2.393*10^-3 -0.583 0;
    0 0 1 0]
B = [0;
    34.703;
    -0.674;
    0]
C = eye(4);
D = [0];

states = {'u' 'w' 'q' 'theta'}
inputs = {'elevator'}
outputs = {'u' 'w' 'q' 'theta'}

sys = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs)
tfsys = tf(sys)
P_Pitch = tfsys(4)

figure(1)
t = [0:0.01:100]; 
[y,t] = step(1/57.3*tfsys,t);
subplot(4,1,1)
plot(t,y(:,1)/57.3,'LineWidth',2)
title('Step response','FontSize',12)
ylabel('p(deg/s)')

subplot(4,1,2)
plot(t,y(:,2)/57.3,'LineWidth',2) 
ylabel('q(deg/s)')

subplot(4,1,3)
plot(t,y(:,3)/57.3,'LineWidth',2)
ylabel('r(deg/s)')

subplot(4,1,4)
plot(t,y(:,4)/57.3,'LineWidth',2)
ylabel('\theta(deg)')
xlabel('second')


figure(2)
[y,t] = step(1/57.3*P_Pitch,t);
plot(t,y/57.3,'LineWidth',2)
ylabel('pitch angle (degree)');
title('Open-loop Step Response');

figure(3)
rlocus(P_Pitch)

sys_cl = feedback(P_Pitch, 1)
figure(4)
pole(P_Pitch)
zero(P_Pitch)
pole(sys_cl)
step(1/57.3*sys_cl,t);
ylabel('Pitch angle \theta (degree)');

figure(5)
margin(P_Pitch)
grid on

figure(6)
nyquist(P_Pitch)
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on

figure(7)
sys_cl = feedback(0.0552 * P_Pitch, 1)
step(1/57.3*sys_cl,t);
ylabel('Pitch angle \theta (degree)');

controlSystemDesigner('rlocus', P_Pitch)

s = tf('s')
sys_c2 = feedback(-0.0043295*(1+1.4*s)*(1+22*s)/s * P_Pitch, 1)
step(1/57.3*sys_c2,600);
margin(sys_c2)

u = ones(size(t));
x0 = [0 0 0 0];
p = [-1 -2 -3 -4]
K = place(A,B,p)
sys_c3 = ss((A-B*K),B,C,D,'statename',states,'inputname',inputs,'outputname',outputs)
tfsys3 = tf(sys_c3)

figure(1)
t = [0:0.01:100]; 
[y,t] = step(1/57.3*tfsys3,t);
subplot(4,1,1)
plot(t,y(:,1)/57.3,'LineWidth',2)
title('Step response','FontSize',12)
ylabel('p(deg/s)')

subplot(4,1,2)
plot(t,y(:,2)/57.3,'LineWidth',2) 
ylabel('q(deg/s)')

subplot(4,1,3)
plot(t,y(:,3)/57.3,'LineWidth',2)
ylabel('r(deg/s)')

subplot(4,1,4)
plot(t,y(:,4)/57.3,'LineWidth',2)
ylabel('\theta(deg)')
xlabel('second')


for i = 1:1:4
    stepinfo(1/57.3 * sys_c3(i))
end
