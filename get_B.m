function B = get_B(x)
    % x is the state vector [u; v; r]
    % params is the structure containing the model parameters
    % The function returns the B matrix which describes how control inputs
    % (thrust and rudder moment) affect the state derivatives (u̇, v̇, ṙ)
    
    % Extract the state variables
    u = x(1); % Surge velocity
    v = x(2); % Sway velocity
    r = x(3); % Yaw rate
    
    % Parameters
        m = 11.8;
    Iden =[-24.2467268577488
    -60.8636190295842
    -80.8105073465124
    44.1820057953471
    -22.4265716252365
    6.56283450663319
    -0.189094539026755
    179.844326909184
    -235.613016365395
    -35.4304364621372
    116.479225159432
    -97.9225095171325
    ];
    Xud = Iden(1);
    IzMinNrd = Iden(3);
    
    
    % Construct the B matrix (control input effects on state derivatives)
    B = zeros(3, 2); % 3 states: u, v, r, and 2 control inputs: tau_u, tau_r
    
    % Influence of thrust (tau_u) and rudder moment (tau_r) on the system
    % Surge (u̇) equation: Influence of tau_u (thrust) on u
    B(1, 1) = 1 / (m - Xud); % tau_u directly affects u̇
    
    % Sway (v̇) equation: Influence of tau_u (thrust) on v
    B(2, 1) = 0; % No direct effect of tau_u on sway in your equations
    
    % Yaw (ṙ) equation: Influence of tau_u (thrust) on r
    B(3, 1) = 0; % No direct effect of tau_u on yaw rate
    
    % Surge (u̇) equation: Influence of tau_r (rudder) on u
    B(1, 2) = 0; % No direct effect of tau_r on surge
    
    % Sway (v̇) equation: Influence of tau_r (rudder) on v
    B(2, 2) = 0; % No direct effect of tau_r on sway
    
    % Yaw (ṙ) equation: Influence of tau_r (rudder) on yaw
    B(3, 2) = 1 / (IzMinNrd); % tau_r directly affects yaw rate
    
    % Note: The terms you have are nonlinear, so this representation
    % assumes the effect is mostly linear, and you'll need to adjust for
    % nonlinearities in the model. This matrix defines how the inputs
    % influence the system dynamics.
end
