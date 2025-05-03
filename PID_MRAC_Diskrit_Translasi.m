function PID_MRAC_Diskrit_Translasi(block)
% Level-2 MATLAB S-Function untuk PID MRAC Translasi (Throttle Control)

    setup(block);
    
end

%% ========== SETUP FUNCTION ==========
function setup(block)
    % Register jumlah input dan output port
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;
    
    % Konfigurasi input port
    block.SetPreCompInpPortInfoToDynamic;
    block.InputPort(1).Dimensions = 7; % 7 input
    block.InputPort(1).DirectFeedthrough = true;
    
    % Konfigurasi output port
    block.SetPreCompOutPortInfoToDynamic;
    block.OutputPort(1).Dimensions = 1; % 1 output (Throttle Control)
    
    % Set sample time
    block.SampleTimes = [0.01 0]; % 10ms sample time
    
    % Register method
    block.RegBlockMethod('Outputs', @Outputs);
end

%% ========== OUTPUT FUNCTION ==========
function Outputs(block)
    % Variabel Persisten (Memori Internal)
    persistent e_last MR_last de_last ie_last;
    
    if isempty(e_last)
        e_last = 0;
        MR_last = 0;
        de_last = 0;
        ie_last = 0;
    end
    
    % Ambil Input dari Simulink
    u = block.InputPort(1).Data;
    X = u(1);  % Input throttle referensi
    Y = u(2);  % Output throttle aktual
    Gain_P = u(3);
    Gain_I = u(4);
    Gain_D = u(5);
    Ts = u(6); % Time step
    t = u(7);  % Waktu
    
    % Model Reference Adaptation
    [MR, MR_last] = HMR(X, MR_last);
    
    % Hitung Error
    e = MR - Y;
    de = (e - e_last) / Ts; % Derivative Error
    ie = ie_last + (e + e_last) * Ts / 2; % Integral Error
    
    % Adaptive PID Gains
    KP = Gain_P * e;
    KI = Gain_I * ie;
    KD = Gain_D * de;
    
    % Output PID MRAC untuk kontrol throttle
    y = KP + KI + KD;
    
    % Update nilai variabel sebelumnya
    e_last = e;
    ie_last = ie;
    de_last = de;
    
    % Kirim output ke Simulink
    block.OutputPort(1).Data = y;
end

%% ========== MODEL REFERENCE FUNCTION ==========
function [y, y_last] = HMR(x_n, y_l)
    y = 0.001249 * x_n + 0.9988 * y_l;
    y_last = y;
end
