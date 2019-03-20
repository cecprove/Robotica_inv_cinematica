%% Algoritmo per l'inversione cinematica con la pseudo-inversa
% in uscita da questo algoritmo ho la Qddot ossia la velocità del giunto e
% lo passo alla funzione analitical per ottenere cosi la q. Bisognerà
% quindi nel blocco analitical fare solo l'integrale

function Qddot = inv_cin_psd(p,theta,Q,a1,a2,a3,a4,J)
        
        q1= Q(1);
        q2= Q(2);
        q3= Q(3);
        q4= Q(4);

     % calcolo della pseudo inversa
        J_pi=(J')*inv(J*(J'));
        P=eye(3)-J_pi*J;
        dw= funzionale_costo();
        X = direct_kinematics_4DoF(q1,q2,q3,q4,a1,a2,a3,a4);
        Xd=[p theta]';
        e = Xd - X; % dove Xd viene dalla traiettoria(ancora non possiamo fare nulla);
        K =[kgain 0; 0 kgain];
        % kgain = tot;
        Ka=[1 0; 0 3];
        % A è il det(Ja*(Ja)')         
        A=2*a1^2*a2^2*cos(q1 + q2)^2*sin(q1)^2 + 2*a1^2*a2^2*sin(q1 + q2)^2*cos(q1)^2 ...
            + 2*a2^2*a3^2*cos(q1 + q2 + q3)^2*sin(q1 + q2)^2 + 2*a2^2*a3^2*sin(q1 + q2 + q3)^2*cos(q1 + q2)^2 +...
            2*a1^2*a3^2*cos(q1 + q2 + q3)^2*sin(q1)^2 + 2*a1^2*a3^2*sin(q1 + q2 + q3)^2*cos(q1)^2 - 4*a2^2*a3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1 + q2) ...
            - 4*a1^2*a3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1)*sin(q1) + 2*a1*a2*a3^2*sin(q1 + q2 + q3)^2*cos(q1 + q2)*cos(q1) +...
            2*a1^2*a2*a3*cos(q1 + q2 + q3)*cos(q1 + q2)*sin(q1)^2 + 2*a1*a2*a3^2*cos(q1 + q2 + q3)^2*sin(q1 + q2)*sin(q1)...
            + 2*a1^2*a2*a3*sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)^2 - 4*a1^2*a2^2*cos(q1 + q2)*sin(q1 + q2)*cos(q1)*sin(q1) - 2*a1^2*a2*a3*cos(q1 + q2 + q3)*sin(q1 + q2)*cos(q1)*sin(q1) ...
            - 2*a1^2*a2*a3*sin(q1 + q2 + q3)*cos(q1 + q2)*cos(q1)*sin(q1) - 2*a1*a2*a3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*cos(q1 + q2)*sin(q1) -...
            2*a1*a2*a3^2*cos(q1 + q2 + q3)*sin(q1 + q2 + q3)*sin(q1 + q2)*cos(q1);
        if(A==0)          
            Qddot=J_pi*(Xddot+K*e)+P*Ka*dw;% dove Xddot è la velocità desiderata dell'organo terminale
        else
            Qddot=J_pi*(Xddot+K*e);
            
%         


        if(abs(X-[p(1);p(2);theta])<1e-3)
            Qddot=[q1 q2 q3 q4];  
        end
    
    end
