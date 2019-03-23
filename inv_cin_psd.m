%% Algoritmo per l'inversione cinematica con la pseudo-inversa
% in uscita da questo algoritmo ho la Qddot ossia la velocità del giunto 

function Qddot = inv_cin_psd(Q,XYd,XYddot,a,joint_lim)

     % calcolo la posa. Successivamente valuto l'errore ossia la differenza
     % tra la posa calcolata con la function cinematica diretta e quella
     % desiderata che otteniamo dalla traiettoria.
     [XY1 XY2 XY3 XY4]=kin_man_rid_progetto(Q,a);
     % dove XY1 è un vettore 3x1 che contiene posizione x, y e orientamento
     % del giunto 1, XY2 del giubnto 2 e cosi via. Quindi XY4 contiene
     % posizione e orientamento dell'organo terminale. 

     %errore
     e=XYd-XY4;
     Ja = J_man_plan_4DoF(Q,a);
     
     % calcolo della pseudo inversa
        J_pi=(Ja')*inv(Ja*(Ja'));
        P=eye(4)-J_pi*Ja;
             
        K=[1 0 0 ;
           0 1 0;
           0 0 1 ]*10;
                    
        if(det(Ja*Ja') == 0) % quindi mi trovo in singolarità cinematica     
            Ka=[1 0; 0 1];
            dw = funzionale_costo(joint_lim,Q,a);
            Qddot=J_pi*(XYddot+K*e)+P*Ka*dw;% dove Xddot è la velocità desiderata dell'organo terminale
        else
            Qddot=J_pi*(XYddot+K*e);
        end

    
    end