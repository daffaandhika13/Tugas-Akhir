function PID_MRAC_Diskrit (block)
% Level-2 MATLAB S-Function for PID MRAC Translasi (Throttle Control)

    setup(block);
end

%% Setup block I/O and timing
function setup(block)
    block.NumInputPorts  = 1;
    block.NumOutputPorts = 1;

    block.SetPreCompInpPortInfoToDynamic;
    block.InputPort(1).Dimensions = 7;
    block.InputPort(1).DirectFeedthrough = true;

    block.SetPreCompOutPortInfoToDynamic;
    block.OutputPort(1).Dimensions = 1;

    block.SampleTimes = [0.01 0];

    block.RegBlockMethod('Outputs', @Outputs);
end

%% Output calculation
function Outputs(block)
    persistent e_last MR_last de_last ie_last

    if isempty(e_last)
        e_last = 0;
        MR_last = 0;
        de_last = 0;
        ie_last = 0;
    end

    u = block.InputPort(1).Data;
    X = u(1);  % Referensi throttle
    Y = u(2);  % Aktual throttle
    Gain_P = u(3);
    Gain_I = u(4);
    Gain_D = u(5);
    Ts = u(6);
    % t = u(7);  % Tidak digunakan

    % === Model Reference ===
    MR = 0.001664 * X + 0.998336 * MR_last;
    MR_last = MR;

    % === Error Calculations ===
    e = MR - Y;
    de = (e - e_last) / Ts;
    ie = ie_last + (e + e_last) * Ts / 2;

    % === PID Calculations ===
    KP = Gain_P * e;
    KI = Gain_I * ie;
    KD = Gain_D * de;

    % === Output Control Signal ===
    y = KP + KI + KD;

    % === Update States ===
    e_last = e;
    de_last = de;
    ie_last = ie;

    % === Output ===
    block.OutputPort(1).Data = y;
end
