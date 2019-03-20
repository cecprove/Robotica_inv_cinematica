% function per definire il funzionale di costo: "funzione di manipolabilità" 
function dw = funzionale_costo() 
 syms q1 q2 q3 q4 'real'
 syms a1 a2 a3 a4 'real'
 Q=[q1 q2 q3 q4];
 
 w=sqrt(2*a1^2*a2^2*cos(q1 + q2)^2*sin(q1)^2 + 2*a1^2*a2^2*sin(q1 + q2)^2*cos(q1)^2 ...
            + 2*a2^2*a3^2*cos(q1 + q2 + q3)^2*sin(q1 + q2)^2 + 2*a2^2*a3^2*sin(q1 + q2 + q3)^2*cos(q1 + q2)^2 +...
            2*a1^2*a3^2*cos(q1 + q2 + q3)^2*sin(q1)^2 + 2*a1^2*a3^2*sin(q1 + q2 + q3)^2*cos(q1)^2 - 4*a2^2*a3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1 + q2) ...
            - 4*a1^2*a3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1)*sin(q1) + 2*a1*a2*a3^2*sin(q1 + q2 + q3)^2*cos(q1 + q2)*cos(q1) +...
            2*a1^2*a2*a3*cos(q1 + q2 + q3)*cos(q1 + q2)*sin(q1)^2 + 2*a1*a2*a3^2*cos(q1 + q2 + q3)^2*sin(q1 + q2)*sin(q1)...
            + 2*a1^2*a2*a3*sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)^2 - 4*a1^2*a2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1) - 2*a1^2*a2*a3*cos(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)*sin(q1) ...
            - 2*a1^2*a2*a3*sin(q1 + q2 + q3)*cos(q1 + q2)*cos(q1)*sin(q1) - 2*a1*a2*a3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1) -...
            2*a1*a2*a3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1));
 for i=1:4
 w1=(Q(i)-q_medio(i))/q_range(i)+w1;
 end
 w1=-(1/4)*w1;
 dw1=simplify(diff(w1, 'q1'))+simplify(diff(w1, 'q2'))+simplify(diff(w1, 'q3'))+simplify(diff(w1, 'q4'));
        
        dw2 = simplify(diff(w, 'q1')) + simplify(diff(w, 'q2')) +simplify(diff(w, 'q3')) + simplify(diff(w, 'q4'));
        dw=dw1+dw2;
 
end
