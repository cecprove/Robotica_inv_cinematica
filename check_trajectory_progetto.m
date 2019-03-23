close all
clear all
clc


load('trajectory_progetto.mat');
load('limiti_giunto.mat');

a1=9;
a2=10;
a3=11;
a4=11;



tf=1;
dt=tf/size(XY,1);
T=0:dt:tf-dt;

XYdot=[0,0,0; diff(XY,1,1)/dt];
% fissiamo un q4 e sfruttiamo la funzione cinematica_inversa_4DoF che
% abbiamo usato per la traiettoria. Tanto questo passaggio ci serve solo
% per calcolare le posizioni iniziali con cui far partire il manipolatore,
% successivamnete implementiamo l'algoritmo di inversione cinematica.
% Scelgo arbitrariamente q4= 10°.
trovato = false;
resolution_q=deg2rad(15);
q4 = joint_lim(4,1);                    
while q4 <= joint_lim(4,2) && ~trovato
    Q0 = cinematica_inversa_4gdl(XY(1,1:2),XY(1,3),[a1,a2,a3,a4], q4)';
    if ~isempty(Q0)
        trovato = true;
        Q=Q0;
    end
    q4 = q4 + resolution_q;
end

XY_IK1=[];
XY_IK2=[];
XY_IK3=[];
XY_IK4=[];
XY_err=[];
joints=[];

% Calcolo il jacobiano simbolico qui fuori dal ciclo for cosi non devo
% calcolarlo per ogni i ma lo faccio una sola volta e basta
% Ja = calcolo_jacobiano_simbolico();
% Ja=[Ja(1:2,:);Ja(6,:)];

for i=1:size(XY,1)
    
    % Inversione cinematica
    Q_dot=inv_cin_psd(Q,XY(i,1:3)',XYdot(i,1:3)',[a1, a2, a3, a4],joint_lim);
    Q=Q+Q_dot*dt;
    
    joints=[joints;Q'];
    [xy_ik1, xy_ik2, xy_ik3 xy_ik4]=kin_man_rid_progetto(Q,[a1, a2, a3, a4]);
    
    XY_IK1=[XY_IK1; xy_ik1'];
    XY_IK2=[XY_IK2; xy_ik2'];
    XY_IK3=[XY_IK3; xy_ik3'];
    XY_IK4=[XY_IK4; xy_ik4'];
    
    
    XY_err=[XY_err; xy_ik4(1:3)'-XY(i,1:3)];
end


figure(1)
subplot(4,1,1)
plot(T,joints(:,1),'-b','Linewidth',4)


subplot(4,1,2)
plot(T,joints(:,2),'-b','Linewidth',4)


subplot(4,1,3)
plot(T,joints(:,3),'-b','Linewidth',4)

subplot(4,1,4)
plot(T,joints(:,4),'-b','Linewidth',4)



figure(2)
subplot(3,1,1)
plot(T,XY_err(:,1),'-b','Linewidth',4)


subplot(3,1,2)
plot(T,XY_err(:,2),'-b','Linewidth',4)


subplot(3,1,3)
plot(T,XY_err(:,3),'-b','Linewidth',4)



MAKE_VIDEO = 1;
if(MAKE_VIDEO)
    motion = VideoWriter(['mov_2D_',datestr(now,30),'.avi']);
    open(motion);
end


figure(5)
for i=1:20:size(XY_IK1,1)

    plot(XY(:,1),XY(:,2),'-k','Linewidth',4)
    hold on
    plot([0 XY_IK1(i,1)],[0 XY_IK1(i,2)],'-r','Linewidth',4)
    plot([XY_IK1(i,1) XY_IK2(i,1)],[XY_IK1(i,2) XY_IK2(i,2)],'-b','Linewidth',4)
    plot([XY_IK2(i,1) XY_IK3(i,1)],[XY_IK2(i,2) XY_IK3(i,2)],'-g','Linewidth',4)
    plot([XY_IK3(i,1) XY_IK4(i,1)],[XY_IK3(i,2) XY_IK4(i,2)],'-y','Linewidth',4)
    
    axis equal
    xlim([-5 35])
    ylim([-10 27])
  

    
    
    if(MAKE_VIDEO)
        F = getframe(gcf);
        writeVideo(motion,F);
    end
    pause(0.5)
    hold off
    
end



if(MAKE_VIDEO)
    close(motion);
end

