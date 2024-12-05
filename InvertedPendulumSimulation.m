function InvertedPendulumSimulation(controller, varargin)
% simulate runs the simulation of a two-wheeled robot
%
%   simulate('FunctionName') uses the controller defined by
%       the function 'FunctionName' - for example, in a file with
%       the name FunctionName.m - in the simulation.
%
%           'moviefile' : a filename (e.g., 'movie.mp4') where, if defined,
%                         a movie of the simulation will be saved
%           'tStop' : the time at which the simulation will stop (a
%                     positive number) - default value is 150
%           'display' : a flag - true (default) or false that, if false,
%                       will not show any graphics and will run the
%                       simulation as fast as possible (not in real-time)
%           'iDelay' : a non-negative integer(default 0) that says how
%                      many time steps sensor data is delayed (each time
%                      step is 1/50 seconds)
%           'shpwGraph' : a flag - true (default) or false that, if false,
%                       will not show any graphs


    % Parse the arguments
    % - Create input parser
    p = inputParser;
    % - Parameter names must be specified in full
    p.PartialMatching = false;
    % - This argument is required, and must be first
    addRequired(p,'controller',@ischar);

    addParameter(p,'moviefile',[],@ischar);
    addParameter(p,'display',true,@islogical);
    addParameter(p,'tStop',10,@(x) isscalar(x) && isnumeric(x) && (x>0));
    addParameter(p,'iDelay',0,@(x) isscalar(x) && isnumeric(x) && (x>=0));
    addParameter(p,'showGraph',true,@(x) isscalar(x) && isnumeric(x) && (x>=0));

    % - Apply input parser
    parse(p,controller,varargin{:});
    % - Extract parameters
    process = p.Results;
    % - Check that the 'controller' function exists
    if (exist(process.controller,'file')~=2)
        error('Controller ''%s'' does not exist.',process.controller);
    end

    % Setup the simulation
    [process,controller] = SetupSimulation(process);
    
    % Run the simulation
    RunSimulation(process,controller);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT WILL CHANGE FOR DIFFERENT PROCESSES
%

function [process,controller] = SetupSimulation(process)
    % - State time.
    process.tStart = 0;
    % - Time step.
    process.tStep = 1/50;
    % - Gravity
    process.g = -9.81;          % m/s^2
  
    % Parameters

    % - Lengths (m)
    process.L = 0.04064;            
    
    % - Masses (kg)
    process.M = 1.5;            % cart weight
    process.m = 1;            % ball weight at tip 
    
    % - Moments of inertia
    process.Iw = (process.m)*process.L^2;     % moment of intertia ball
    

    % friction
    process.dcart = 0.3;
    process.dpendulum = 0;
    

    % - EOM
    filename = 'InvertedPendulumSimulation_EOMs.mat';
    if (exist(filename,'file')==2)
        fprintf(1,'Loading EOMs from file (delete %s to start fresh).\n',filename);
        load(filename);
    else
        [symEOM,numEOM] = GetEOM(process.g,...
                                 process.Iw,...
                                 process.M,process.m,...
                                 process.L);
	    fprintf(1,'Saving EOMs to file (load %s to work with them).\n',filename);
	    save('Project05_EOMs.mat','symEOM','numEOM');
    end
    process.symEOM = symEOM;
    process.numEOM = numEOM;
    


    % - Maximum force
    process.forceMax = 10;
    
    % DEFINE VARIABLES
    
    % Time
    process.t = 0;
    % States
    process.x = 0;%initialCondition.x;
    process.v = 0;
    process.theta = pi/4;%initialCondition.theta;
    process.omega = 0;

    %window size
    process.w = 10;
    process.h = 5;
   
    % cart
    process.cartSize = 1;
    process.cartX = [-process.cartSize, process.cartSize, process.cartSize, -process.cartSize];
    process.cartY = [-process.cartSize, -process.cartSize, process.cartSize, process.cartSize];
    
    % graphs
    process.graph.x1 = zeros(1, process.tStop / process.tStep);
    process.graph.x2 = zeros(1, process.tStop / process.tStep);
    process.graph.x3 = zeros(1, process.tStop / process.tStep);
    process.graph.x4 = zeros(1, process.tStop / process.tStep);

    process.graph.x3hat = zeros(1, process.tStop / process.tStep);

    process.graph.xhat = zeros(4, process.tStop / process.tStep);


    process.graph.control = zeros(1, process.tStop / process.tStep);
    
    process.graph.i = 1;

    % DEFINE CONTROLLER

    % Functions
    % - get handles to user-defined functions 'init' and 'run'
    controller = eval(process.controller);
    controller.name = process.controller;
    
    % Parameters
    % - define a list of constants that will be passed to the controller
    names = {'tStep','symEOM','numEOM'};
    % - loop to create a structure with only these constants
    controller.parameters = struct;
    for i=1:length(names)
        controller.parameters.(names{i}) = process.(names{i});
    end
   
    % Parameters
    % - define a list of constants that will be passed to the controller
    names = {'tStep','forceMax','symEOM','numEOM'};
    % - loop to create a structure with only these constants
    controller.parameters = struct;
    for i=1:length(names)
        controller.parameters.(names{i}) = process.(names{i});
    end

    % Storage
    controller.data = struct;
    % References
    controller.references = GetReferences(process);
    % Sensors
    % Sensors
    sensors = CreateSensors(process);
    for i=0:process.iDelay
        process.sensors_delayed{i+1} = sensors;
    end
    controller.sensors = GetSensors(process);

    % Actuators
    controller.running = true;
    tic
    try
        [controller.actuators,controller.data] = ...
            controller.init(controller.sensors, ...
                               controller.references, ...
                               controller.parameters, ...
                               controller.data);
    catch exception
        warning(['The ''init'' function of controller\n     ''%s''\n' ...
                 'threw the following error:\n\n' ...
                 '==========================\n' ...
                 '%s\n', ...
                 '==========================\n\n' ...
                 'Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'], ...
                 controller.name, ...
                 getReport(exception));
	    controller.actuators = ZeroActuators();
        controller.running = false;
    end
    if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
        warning(['The ''init'' function of controller\n     ''%s''\n' ...
                 'did not return a structure ''actuators'' with the right\n' ...
                 'format. Turning off controller and setting all\n' ...
                 'actuator values to zero.\n'],controller.name);
        controller.actuators = ZeroActuators();
        controller.running = false;
    end
    controller.tComputation = toc;
end

function [symEOM,numEOM] = GetEOM(g,Iw,M, m, L)
    syms x1 x2 x3 x4 x v theta omega u F;
    l = L;
    xdot = [
        x2; 
        (4*sin(x3)*m^2*x4^2 - x2*sin(2*x3)*m^2*x4 - 2*g*sin(2*x3)*m^2 + 4*M*sin(x3)*m*x4^2 + 8*u*m + 8*M*u)/(16*M*m - m^2*cos(2*x3) + 8*M^2 + 7*m^2); 
        x4; 
        (m*(4*g*m*sin(x3) - 2*u*cos(x3) + 4*M*g*sin(x3) - m*x4^2*cos(x3)*sin(x3) + 2*M*x2*x4*sin(x3) + 2*m*x2*x4*sin(x3)))/(l*(4*M^2 + 8*M*m - m^2*cos(x3)^2 + 4*m^2))
           ];
    
   
    symEOM.f = subs(xdot, [x1, x2, x3, x4, u], [x v theta omega, F]);
    % Numeric
    numEOM.f = matlabFunction(symEOM.f,'Vars',[x v theta omega, F]);
end

function references = GetReferences(process)
    references = struct;
end

function sensors = CreateSensors(process)
    sensors.t = process.t;
    sensors.theta = process.theta;
    sensors.omega = process.omega;

    sensors.x = process.x;
    sensors.v = process.v;

    % Add noise
    %   (nothing)
end

function sensors = GetSensors(process)
    sensors = process.sensors_delayed{1};
end

function iscorrect = CheckActuators(actuators)
    iscorrect = false;
    if all(isfield(actuators,{'force'}))&&(length(fieldnames(actuators))==1)
        if isnumeric(actuators.force)
            if isscalar(actuators.force)
                if (~isnan(actuators.force))&&(~isinf(actuators.force))
                    iscorrect = true;
                end
            end
        end
    end
end

function actuators = ZeroActuators()
    actuators = struct('force',0);
end

%
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FUNCTIONS THAT (I HOPE) WILL REMAIN THE SAME FOR ALL PROCESSES
%

function RunSimulation(process,controller)

    % START-UP
    
    % Create empty figure.
    fig = [];

    % Flag to stop simulation on keypress.
    global done
    done = false;
     
    % Start making movie, if necessary.
    if (~isempty(process.moviefile))
        myV = VideoWriter(process.moviefile,'MPEG-4');
        myV.Quality = 100;
        myV.FrameRate = 1/process.tStep;
        open(myV);
    end

    % LOOP
    % Loop until break.
    tStart = tic;
    while (1)

        % Update figure (create one if fig is empty).
        if (process.display)
            fig = UpdateFigure(process,controller,fig);
        end
        
         % If making a movie, store the current figure as a frame.
        if (~isempty(process.moviefile))
            frame = getframe(gcf);
            writeVideo(myV,frame);
        end
    
        % Stop if time has reached its maximum.
        if ((process.t>=process.tStop)||done)
            break;
        end
        % Update process (integrate equations of motion).
        [process,controller] = UpdateProcess(process,controller);
    
        % Wait if necessary, to stay real-time.
        if (process.display)
            while (toc(tStart)<process.t-process.tStart)
                % Do nothing
            end
        end
    end
end

function fig = UpdateFigure(process,controller,fig)
    model.L = process.L;
    model.x = process.x;
    model.theta = process.theta;
    model.cartX = process.cartX;
    model.cartY = process.cartY;
    model.cartSize = process.cartSize;
    L = model.L;
    x = model.x;
    theta = model.theta;
    cartX = model.cartX;
    cartY = model.cartY;
    cartSize = model.cartSize;
%      Calculate new position
    px = L*cos(pi/2 - theta);
    py = L*sin(pi/2 - theta);

    if (isempty(fig))
        % CREATE FIGURE

        % Clear the current figure.
        clf;

        % Create an axis for text (it's important this is in the back,
        % so you can rotate the view and other stuff!)
        fig.text.axis = axes('position', [0.1 0.1 0.8 0.8]);
        hold on;
        axis off;
        % font size
        fs = 14; 
        if (controller.running)
            status = 'ON';
            color = 'g';
        else
            status = 'OFF';
            color = 'r';
        end
        fig.text.status=text(0.05,0.975,...
            sprintf('CONTROLLER: %s',status),...
            'fontweight','bold','fontsize',fs,...
            'color',color,'verticalalignment','top');
        fig.text.time=text(0.05,0.9,...
            sprintf('time: %6.2f\n',process.t),...
            'fontsize',fs,'verticalalignment','top','fontname','monaco');

        set(gcf,'renderer','opengl');
        set(gcf,'color','w');
        
        
        fig.view0.axis = axes('position',[0 0 .5 .5]);
        axis equal;
        axis([-5, 5, -5, 5])
        hold on;
        axis off;

        fig.view0.model = drawModel([], model);
    
%         fig.view0.model.ground1 = line([-process.w process.w], [-process.cartSize -process.cartSize], 'LineWidth', 2, 'Color', 'black');
       
        
        x = linspace(-10, 1, 10);      % x data
        y = x*0 -process.cartSize;                     % y data
        
        % Create a colormap from red to pink
        numSegments = length(x) - 1;    % Number of line segments
        colors = [linspace(1, 1, numSegments)', linspace(0, 0.75, numSegments)', linspace(0, 0.75, numSegments)'];
        
        
        hold on;
        
        % Plot each line segment with a different color
        for i = 1:numSegments
            fig.view0.model.ground(i) = plot(x(i:i+1), y(i:i+1), 'Color', colors(i, :), 'LineWidth', 2);
        end


        if(process.showGraph)
            fig.view1.axis = axes('position',[.6 .1 .4 .4]);
            axis normal
            axis([0 process.tStop -process.forceMax process.forceMax]);
            hold on;
            axis on;
            
            % Initialize plot for control graph with initial data (ensure lengths match)
            fig.view1.graph.control = plot([0], [0], 'b', 'LineWidth', 2);
            xlabel('t') 
            ylabel('u') 
       


            fig.view2.axis = axes('position',[.6 .5 .4 .4]);
            axis auto
%             axis([0 process.tStop -1.5*pi 1.5*pi]);
            hold on;
            axis on;
            
            % Initialize plot for control graph with initial data (ensure lengths match)
            fig.view2.graph.x3 = plot([0], [0], 'b', 'LineWidth', 2);
            xlabel('t') 
            ylabel('\theta') 


            fig.view2.graph.x3hat = plot([0], [0], '--r', 'LineWidth', 2);
            xlabel('t') 
            ylabel('hat t') 
            legend('- \theta(t)', '- hat t(t)')
           
%             fig.view2.graph.x4 = plot([0], [0], 'b', 'LineWidth', 2);

        end
    else
        
        fig.view0.model = drawModel(fig.view0.model, model);
         set(fig.view0.axis, 'XLim', [model.x-5 model.x+5]);
        set(fig.text.time,'string',sprintf('time: %6.2f\n',process.t));
        if (controller.running)
            status = 'ON';
            color = 'g';
        else
            status = 'OFF';
            color = 'r';
        end
        set(fig.text.status,'string',sprintf('CONTROLLER: %s',status),'color',color);
        
        % Update control graph with new time and control data

    
        if(process.showGraph)
            T = 0:process.tStep:process.tStop;
       
     
   
            set(fig.view1.axis, 'YLim', [min(process.graph.control)-5 max(process.graph.control)+5]);
            set(fig.view1.graph.control, 'XData', T(1:process.graph.i-1), 'YData', process.graph.control(1:process.graph.i-1));

              
            set(fig.view2.axis, 'XLim', [0 process.tStop]);
            set(fig.view2.graph.x3, 'XData', T(1:process.graph.i-1), 'YData', process.graph.x3(1:process.graph.i-1));
            set(fig.view2.graph.x3hat, 'XData', T(1:process.graph.i-1), 'YData', process.graph.x3hat(1:process.graph.i-1));

%             set(fig.view2.graph.x4, 'XData', T(1:process.graph.i-1), 'YData', process.graph.x4(1:process.graph.i-1));

        end
     end
    drawnow;
end

function modelfig = drawModel(modelfig, model)
    L = 4;%model.L * 100;
    x = model.x;
    theta = model.theta;
    cartX = model.cartX;
    cartY = model.cartY;
    cartSize = model.cartSize;
%      Calculate new position
    px = L*cos(pi/2 - theta);
    py = L*sin(pi/2 - theta);

    if isempty(modelfig)
        modelfig.cart = patch(cartX + [x, x, x, x], cartY, 'b');
       
        % Pendulum plot object
         modelfig.pendulumLine = line([x x+px], [cartSize cartSize+py], ...
             'LineWidth', 2, 'Color', 'g');
         modelfig.pendulumMass = plot(x+px, cartSize+py, ...
             'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    else
        % Update plot

         set(modelfig.cart, 'XData', cartX + [x, x, x, x]);         
         set(modelfig.pendulumLine, 'XData', [x x+px], 'YData', [cartSize cartSize+py]);
         set(modelfig.pendulumMass, 'XData', x+px, 'YData', cartSize+py);
    
    end
end


function [process, controller] = UpdateProcess(process, controller) 
        

        [t0,x] = Get_TandX_From_Process(process);
        
        u = GetInput(process, controller.actuators);
        
        [t, x] = ode45(@(t, x) GetXDot(t, x, u, process), [t0, t0+process.tStep], x);

        process = Get_Process_From_TandX(t(end),x(end,:)',process);
        
        % add to graphs

        process.graph.x1(process.graph.i) = process.x;
        process.graph.x2(process.graph.i) = process.v;
        process.graph.x3(process.graph.i) = process.theta;
        process.graph.x4(process.graph.i) = process.omega;
        
        process.graph.x3hat(process.graph.i) = controller.data.xhat(3);
        process.graph.control(process.graph.i) = u;

        

        process.graph.i = process.graph.i + 1;
        % Get reference values
        controller.references = GetReferences(process);
        
        % Get sensor values
        controller.sensors = GetSensors(process);
         
        process.t = process.t + process.tStep;
        % Get actuator values (run controller)
        if (controller.running)
            tic
            try
                [controller.actuators,controller.data] = ...
                    controller.run(controller.sensors, ...
                                      controller.references, ...
                                      controller.parameters, ...
                                      controller.data);
            catch exception
                warning(['The ''run'' function of controller\n     ''%s''\n' ...
                         'threw the following error:\n\n' ...
                         '==========================\n' ...
                         '%s\n', ...
                         '==========================\n\n' ...
                         'Turning off controller and setting all\n' ...
                         'actuator values to zero.\n'], ...
                         controller.name, ...
                         getReport(exception));
                controller.actuators = ZeroActuators();
                controller.running = false;
            end
            if (~isstruct(controller.actuators) || ~CheckActuators(controller.actuators))
                warning(['The ''run'' function of controller\n     ''%s''\n' ...
                         'did not return a structure ''actuators'' with the right\n' ...
                         'format. Turning off controller and setting all\n' ...
                         'actuator values to zero.\n'],controller.name);
                controller.actuators = ZeroActuators();
                controller.running = false;
            end
            controller.tComputation = toc;
        else
            controller.tComputation = 0;
        end
end


function u = GetInput(process,actuators)
    
    u = [actuators.force];

    if (abs(actuators.force) > process.forceMax)
        u = [max(min(actuators.force, process.forceMax),-process.forceMax)];
    end
end


function xdot = GetXDot(t,x,u,process)
    M = process.M;
    m = process.m;
    l = process.L;
    g = process.g;
    x1 = x(1,1);
    x2 = x(2,1);
    x3 = x(3,1);
    x4 = x(4,1);

    dc = process.dcart;
    dp = process.dpendulum; 
    
    xdot = [
            x2; 
            (4*sin(x3)*m^2*x4^2 - x2*sin(2*x3)*m^2*x4 - 2*g*sin(2*x3)*m^2 + 4*M*sin(x3)*m*x4^2 + 8*u*m + 8*M*u)/(16*M*m - m^2*cos(2*x3) + 8*M^2 + 7*m^2); 
            x4; 
            (m*(4*g*m*sin(x3) - 2*u*cos(x3) + 4*M*g*sin(x3) - m*x4^2*cos(x3)*sin(x3) + 2*M*x2*x4*sin(x3) + 2*m*x2*x4*sin(x3)))/(l*(4*M^2 + 8*M*m - m^2*cos(x3)^2 + 4*m^2))
           ];
    
end

function [t,x] = Get_TandX_From_Process(process)
    t = process.t;
    x = [process.x;
         process.v;
         process.theta;
         process.omega];
end

function process = Get_Process_From_TandX(t,x,process)
    process.x = x(1,1);
    process.v = x(2,1);
    process.theta = x(3,1);
    process.omega = x(4,1);
    
    % Update sensors
    process = UpdateSensors(process);
end

function process = UpdateSensors(process)
    sensors = CreateSensors(process);
    process.sensors_delayed = process.sensors_delayed(2:end);
    process.sensors_delayed{end+1} = sensors;
end



function [actuators, data] = runControlSystem(sensors,references,parameters,data)

    actuators.force = 1;
end