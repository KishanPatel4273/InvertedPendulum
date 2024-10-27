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
    
    syms x v theta omega F;
     
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
    
    data.t = 0;
    data.ze = ze;
    data.ur = ue;
    data.A = A;
    data.B = B;

    data.K = place(A, B, [-.01, -.1, -6, -.4]);
end

%
% STEP #4: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
function [actuators,data] = runControlSystem(sensors,references,parameters,data)
%%%%%%%%%% UPDATE YOUR CONTROLLER AND OBSERVER HERE %%%%%%%%%% 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    x = [sensors.x; sensors.v; sensors.theta; sensors.omega];

    actuators.force = - data.K * (x - data.ze);

    data.t = data.t + parameters.tStep;
end