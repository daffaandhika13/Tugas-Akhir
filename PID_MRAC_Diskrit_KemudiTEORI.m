% Main Function
function y = PID_MRAC_DISKRIT(u)
%variabel data sebelum
persistent e_last eu_last MR_last de_last ie_last Y_last TFI_last TFP_last TFD_last TFIE_last TFPE_last TFDE_last ITFPE_last ITFIE_last ITFDE_last
%Variabel data sekarang
% X(input),Y(input),e,de,ie,MR,TFI,TFP,TFD,TFPE,TFIE,TFDE,ITFIE,ITFPE,ITFDE,KI,KP,KD,I,P,D,Uc(output)
%persistent X Y Gain_P Gain_I Gain_D Ts;
% e de ie MR TFI TFP TFD TFPE TFIE TFDE ITFIE ITFPE ITFDE KI KP KD I P D
%persistent e de ie MR TFI TFP TFD TFPE TFIE TFDE ITFIE ITFPE ITFDE KI KP KD I P D
%Deklarasi konstanta


%Ambil Data Input
X = u(1);
Y = u(2);
Gain_P = u(3);
Gain_I = u(4);
Gain_D = u(5);
Ts = u(6);
t = u(7);
%Ts = 0.01;
if t==0
    e_last = 0;
    eu_last = 0;
    MR_last = 0; 
    de_last = 0; 
    ie_last = 0;
    Y_last = 0;
    TFI_last = 0; 
    TFP_last = 0; 
    TFD_last = 0;
    TFIE_last = 0;
    TFPE_last = 0;
    TFDE_last = 0;
    ITFPE_last = 0;
    ITFIE_last = 0;
    ITFDE_last = 0;
end
%Hitung model reference
[MR,MR_last] = HMR(X,MR_last);

%Hitung Error
e = MR - Y; %Sebenarnya e = Y - MR, namun dikasih minus menjadi e=(-e)

%Hitung Error 
eu= X - Y;

%Hitung derivative error
de = Derivative_Diskrit(Y,Y_last,Ts);

%Hitung Integral error
[ie,ie_last] = Integral_Diskrit(eu,eu_last,ie_last,Ts);

%hitung kontrol proporsional
    %hitung output TFP(
    [TFP,TFP_last] = HTFP(eu,eu_last,TFP_last);
    %Hitung TFPE
    TFPE = TFP*e;
    %hitung integral TFPE
    [ITFPE,ITFPE_last] = Integral_Diskrit(TFPE,TFPE_last,ITFPE_last,Ts);
    %hitung KP
    KP = ITFPE*Gain_P;
    %Hitung Proporsional Control
    P = KP*eu;
    %Post Iteration
    TFPE_last = TFPE;
%hitung kontrol Integral
    %hitung output TFI
    [TFI,TFI_last] = HMR(eu,TFI_last);
    %hitung TFIE
    TFIE = TFI*e;
    %hitung integral TFIE
    [ITFIE,ITFIE_last] = Integral_Diskrit(TFIE,TFIE_last,ITFIE_last,Ts);
    %hitung KI
    KI = ITFIE*Gain_I;
    %hitung Integral Control
    I = KI*ie;
    %Post Iteration
    TFIE_last = TFIE;
%hitung kontrol derivative
    %hitung TFD
    [TFD,TFD_last] = HTFP(de,de_last,TFD_last);
    %hitung TFDE
    TFDE = TFD*e;
    %hitung ITFDE
    [ITFDE,ITFDE_last] = Integral_Diskrit(TFDE,TFDE_last,ITFDE_last,Ts);
    %hitung KD
    KD = ITFDE*Gain_D;
    %hitung Derivetive Control
    D = -KD*de;
    %Post Iteration
    TFDE_last = TFDE;
%Post Iteration error
e_last = e;
eu_last = eu;
e_last = ie;
de_last = de;
Y_last = Y;
%hitung aksi kontrol
y = P+I+D;
end
% fungsi Hitung output model reference
function [y,y_last] = HMR(x_n,y_l) %G=1/(9s+1)
y = 0.00111*x_n+0.9989*y_l;
y_last = y;
end
%fungsi Hitung output TFP
function [y,y_last] = HTFP(x_n,x_l,y_l)
y = 0.1111*x_n-0.1111*x_l+0.9989*y_l;
y_last=y;
end
%fungsi hitung Integral
function [y,Y_last] = Integral_Diskrit(U,u_last,y_last,Ts)
y = y_last+(U+u_last)*Ts/2;
Y_last=y;
end
%fungsi hitung derivative
function y = Derivative_Diskrit(U,u_last,Ts)
y = (U-u_last)/Ts;
end