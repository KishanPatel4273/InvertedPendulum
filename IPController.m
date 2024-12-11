% Designed for use with Project05.m
% Run with 
%   Project05('ControllerLastName')

% 
% STEP #1: Rename this file by replacing LastName with your last name
% in the file name ControllerLastName.m.)

%
% STEP #2: Rename, but DO NOT MODIFY, the function ControllerLastName by 
% replacing LastName with your last name.
function func = Controller
    % Do not modify this function.
    func.init = @initControlSystem;
    func.run = @runControlSystem;
end

%
% STEP #3: Modify, but DO NOT RENAME, the function initControlSystem. It is
% called once, before the simulation loop starts.  If there are quantities 
% needed in your control function and you only want to define them once 
% (e.g., to improve your code's speed) you can define them here and store 
% them in the structure "data", which is supplied as an input to the 
% function runControlSystem below.  For example, to store the variable
% a = 4 in the structure data, you can write: data.a = 4;
% The function should output your data file and the initial values of the
% actuation to apply (i.e., actuators.tauR and actuators.tauL).
% The variables in sensors, references, and parameters are described below
function [actuators,data] = initControlSystem(sensors,references,parameters,data)
% INTERFACE
%
%   sensors
%       .x                (postion of cart)
%       .v                (velocity of cart)
%       .theta            (angle of pendulum)
%       .omega            (angular velocity of pendulum)
%
%   references
%       (none)
%
%   parameters
%       .dt      (time step)
%       .symEOM     (nonlinear EOMs in symbolic form)
%       .numEOM     (nonlinear EOMs in numeric form)
%
%   data
%       .whatever   (yours to define - put whatever you want into "data")
%
%   actuators
%       .force       (force applied on cart horizontally (+x) )

% define the state, control, and output in the nonlinear system
    
    syms x v theta omega F delta;
     
    z = [x; v; theta; omega];
    u = [F];

%%%%%%%%%% SELECT YOUR EQUILIBRIUM POINT HERE %%%%%%%%%%
ze = [0; 0; 0; 0];
ue = [0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% define the nonlinear equations of motion

    f_sys = parameters.symEOM.f;
    dfdz = jacobian(f_sys, z);
    dfdv = jacobian(f_sys, u);
    
    A = double(subs(dfdz, [z; u], [ze; ue]));
    B = double(subs(dfdv, [z; u], [ze; ue]));
%%%%%%%%%% DO YOUR CONTROLLER AND OBSERVER DESIGN HERE %%%%%%%%%%
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % define control to apply in the nonlinear system
    actuators.force = 0;
    
    % save any data that will be needed later
    
    data.dt = parameters.tStep;
    data.t = 0;
    data.ze = ze;
    data.ue = ue;
    data.A = A;
    data.B = B;
    C = [1 0 0 0];
%     C = [1, 0, 0, 0;
%          0, 0, 1, 0];
    data.C = C;
    
    % kalman filter
    data.xhat = [0; 0; pi; 0];

    Vd = .1*diag([1, 1, 1, 1]);
    Vn = 1*diag([1]);
    
    data.Kf = lqr(A', C', Vd, Vn)';
%     data.Kf = [10; 10; 10; 10];
    data.Kf

    % proportional
    Q = diag([1 1 10 100]);
    R = diag([10]);
%    data.K = place(A, B, [-1, -1.2, -3, -4]);
    data.K = lqr(data.A, data.B, Q, R);
    data.K
    % intergal action
    data.w = [0; 0; 0; 0];
    data.Kint = .01*[1 1 1 1];
end

%
% STEP #4: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
function [actuators,data] = runControlSystem(sensors,references,parameters,data)
%%%%%%%%%% UPDATE YOUR CONTROLLER AND OBSERVER HERE %%%%%%%%%% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    theta = sensors.theta;
    theta =  mod(theta + pi, 2*pi) - pi;
   
    x = [sensors.x; sensors.v; theta; sensors.omega];

    % control
    u = data.ue - data.K * (x - data.ze) - data.Kint * data.w;

    maxForce = 10;
    u = min(maxForce, max(-maxForce, u));

    % measurement
    y = data.C*x;
       
    
    data.xhat(3) = mod(data.xhat(3) + pi, 2*pi) - pi; 

    % update
    xhatdot = @(t, xx) data.A*(xx-data.ze) + data.B*(u-data.ue) + data.Kf*(y-data.C*xx);
    [tt, xxhat] = ode45(xhatdot, [data.t, data.t+data.dt], data.xhat);
    data.xhat = xxhat(end,:)';
    
   
    data.w = data.w + (x - data.ze) * data.dt;

   
    actuators.force = u;
    
    data.t = data.t + data.dt;
end