function build_problem_1_simulink(root)
% Create the actual .slx model only when Simulink is installed and licensed.
if nargin<1,root=fileparts(mfilename('fullpath'));end
if exist('new_system','file')~=2 || ~license('test','Simulink')
 fprintf('Simulink is unavailable. Skipping .slx model generation.\n');return
end
mdl='problem_1_nonlinear_pendulum';
if bdIsLoaded(mdl),close_system(mdl,0);end
new_system(mdl);open_system(mdl);ws=get_param(mdl,'ModelWorkspace');
assignin(ws,'x10',pi/6);assignin(ws,'x20',0);assignin(ws,'J',1);assignin(ws,'pulse_width',.02);assignin(ws,'pulse_period',4);
add_block('simulink/Sources/Step',[mdl '/Unit step'],'Time','0','Before','0','After','1','Position',[30 50 65 80]);
add_block('simulink/Sources/Pulse Generator',[mdl '/Pulse train'],'Amplitude','J/pulse_width','Period','pulse_period','PulseWidth','100*pulse_width/pulse_period','Position',[30 130 65 160]);
add_block('simulink/Signal Routing/Manual Switch',[mdl '/Input select'],'sw','1','Position',[125 60 155 120]);
add_block('simulink/Math Operations/Sum',[mdl '/Sum'],'Inputs','+-','Position',[220 75 245 105]);
add_block('simulink/Continuous/Integrator',[mdl '/Velocity'],'InitialCondition','x20','Position',[295 75 330 105]);
add_block('simulink/Continuous/Integrator',[mdl '/Angle'],'InitialCondition','x10','Position',[390 75 425 105]);
add_block('simulink/Math Operations/Trigonometric Function',[mdl '/sin theta'],'Operator','sin','Position',[380 215 425 245]);
add_block('simulink/Math Operations/Gain',[mdl '/g over l'],'Gain','10','Position',[285 215 325 245]);
add_block('simulink/Signal Routing/Mux',[mdl '/States'],'Inputs','2','Position',[485 65 490 125]);
add_block('simulink/Sinks/Scope',[mdl '/Scope'],'Position',[545 75 580 110]);
add_block('simulink/Sinks/To Workspace',[mdl '/Save states'],'VariableName','states','SaveFormat','Structure With Time','Position',[540 145 630 175]);
add_line(mdl,'Unit step/1','Input select/1','autorouting','on');add_line(mdl,'Pulse train/1','Input select/2','autorouting','on');
add_line(mdl,'Input select/1','Sum/1');add_line(mdl,'Sum/1','Velocity/1');add_line(mdl,'Velocity/1','Angle/1');
add_line(mdl,'Angle/1','sin theta/1','autorouting','on');add_line(mdl,'sin theta/1','g over l/1');add_line(mdl,'g over l/1','Sum/2','autorouting','on');
add_line(mdl,'Angle/1','States/1','autorouting','on');add_line(mdl,'Velocity/1','States/2','autorouting','on');
add_line(mdl,'States/1','Scope/1');add_line(mdl,'States/1','Save states/1','autorouting','on');
set_param(mdl,'Solver','ode45','StopTime','12','MaxStep','0.002','RelTol','1e-8');
save_system(mdl,fullfile(root,[mdl '.slx']));
end
