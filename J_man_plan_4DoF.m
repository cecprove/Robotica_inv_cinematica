%% Algoritmo di inversione cinematica con l'inversa dello jacobiano per manipolatore planare a 4DoF
% Dobbiamo imporre quindi necessariamente delle condizioni iniziali per la
% posizione dei 4 giunti. 
function J= J_man_plan_4DoF(Q,a1,a2,a3,a4)
q1=Q(1);
q2=Q(2);
q3=Q(3);
q4=Q(4);
% calcolo del jacobiano
J=[-a1*sin(q1)-a2*sin(q1+q2)-a3*sin(q1+q2+q3)-a4*sin(q1+q2+q3+q4) -a2*sin(q1+q2)-a3*sin(q1+q2+q3)-a4*sin(q1+q2+q3+q4) ...
    -a3*sin(q1+q2+q3)-a4*sin(q1+q2+q3+q4) -a4*sin(q1+q2+q3+q4);
    a1*cos(q1)+a2*cos(q1+q2)+a3*cos(q1+q2+q3)+a4*cos(q1+q2+q3+q4)  a2*cos(q1+q2)+a3*cos(q1+q2+q3)+a4*cos(q1+q2+q3+q4) ...
    a3*cos(q1+q2+q3)+a4*cos(q1+q2+q3+q4) a4*cos(q1+q2+q3+q4); 1 1 1 1];

end