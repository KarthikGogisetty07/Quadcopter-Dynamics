function result = simulate(controllers, desired_state, init_state, t,tf,dt)
    % Physical constants
    % t = initial time 
    % tf = final time 
    g = 9.81;
    m = 0.468;
    L = 0.225;
    k = 3*10^-6;                                          % thrust constant
    b = 1.1*10^-7;                                        % drag constant
    I = [0.05, 0, 0
        0, 0.05, 0
        0, 0, 0.05];
    k_d = 0.25;
    
   % time setup 
    ts = t:dt:tf;
    N = numel(ts);
    % numel gives the number of elements in an array.
    %Number of focus points = N
    
    % Output values, recorded as the simulation runs.
    xout = zeros(3, N);
    xdotout = zeros(3, N);
    thetaout = zeros(3, N);
    thetadotout = zeros(3, N);
    inputout = zeros(4, N);

    % Struct given to the controller to run simulations
    controller_params = struct('dt', dt, 'I', I, 'k', k, 'L', L, 'b', b, 'm', m, 'g', g);

    % Initial system state.
    x = init_state.x;
    xdot = init_state.xdot;
    theta = init_state.theta;
    thetadot = init_state.thetadot;

    ind = 0;
    for t = ts
        ind = ind + 1;
            
        [i, controller_params] = resolve_control_signals(controllers, controller_params, desired_state, theta, thetadot, x(3), xdot(3));
        % controller_params = State of Quad 
        % i = Control signal to the Quads 
        % This Function is called in resolve_control_signal.
        
        %======================================================================
        % Compute forces, torques, and accelerations
        omega = thetadot2omega(thetadot, theta);
        a = acceleration(i, theta, xdot, m, g, k, k_d);
        omegadot = angular_acceleration(i, omega, I, L, b, k);

        % Progress the system state
        omega = omega + dt * omegadot;
        thetadot = omega2thetadot(omega, theta); 
        theta = theta + dt * thetadot;
        xdot = xdot + dt * a;
        x = x + dt * xdot;

        % Store simulation state for output
        xout(:, ind) = x;
        xdotout(:, ind) = xdot;
        thetaout(:, ind) = theta;
        thetadotout(:, ind) = thetadot;
        inputout(:, ind) = i;
    end

    % Put all simulation variables into an output struct
    result = struct('x', xout, 'theta', thetaout, 'vel', xdotout, ...
                    'angvel', thetadotout, 't', ts, 'dt', dt, 'input', inputout);
end

% ===========================DYNAMICS==================================== %

% Compute Thrust given current inputs (thrust force on each motor) and thrust coefficient
function T = thrust(inputs, k)
    T = [0; 0; k * sum(inputs)];
end

% Compute torques given current inputs, length of the drone arm, drag coefficient and thrust coefficient
function tau = torques(inputs, L, b, k)
    tau = [
        L * k * (inputs(4) - inputs(2)) % roll
        L * k * (inputs(3) - inputs(1)) % pitch
        b * (inputs(4) - inputs(3) + inputs(2) - inputs(1)) % yaw
    ];
end
        
function a = acceleration(inputs, angles, xdot, m, g, k, k_d)
    gravity = [0; 0; -g];
    R = Rotation(angles);
    T = R * thrust(inputs, k);
    % aerodynamic drag is considered can ignore to design a simple system. 
    Fd = -k_d * xdot;                                     
    a = gravity + (1 / m) * T + Fd;
end

function omegadot = angular_acceleration(inputs, omega, I, L, b, k)
    tau = torques(inputs, L, b, k);
    omegadot = I\(tau - cross(omega, I * omega));
end

% Omega and thetadot are angular velocities 


% Convert derivatives of roll, pitch, yaw to omega.
function omega = thetadot2omega(thetadot, angles)
    phi = angles(1);
    theta = angles(2);
    
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];

    omega = W * thetadot;
end

% Convert omega to roll, pitch, yaw derivatives
function thetadot = omega2thetadot(omega, angles)
    phi = angles(1);
    theta = angles(2);
    
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];

    thetadot = W\omega;
end