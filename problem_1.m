function problem_1
% PROBLEM 1: nonlinear pendulum, step response, pulses and live animation.
% Angle is measured from the DOWNWARD vertical. Input u=tau/(m*l^2).
root=fileparts(mfilename('fullpath')); out=fullfile(root,'outputs');
if ~exist(out,'dir'), mkdir(out); end
diary(fullfile(out,'matlab_run.log')); diary on
g=10; l=1; w=sqrt(g/l); ep=0.05; U=1;
IC=[pi/6 0;0 pi/w;pi/2 0;2*pi+ep 0;2*pi-ep 0;0 0];
names={'I: pi/6, 0','II: 0, pi/omega','III: pi/2, 0', ...
    'IV: 2pi+epsilon, 0','V: 2pi-epsilon, 0','VI: 0, 0'};
colors=lines(6); t=(0:0.01:12)'; opts=odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',0.01);
Z=cell(6,1); Zfree=Z; errors=zeros(6,1);
fprintf('PROBLEM 1 | g=10, l=1, omega=%.9f, epsilon=%.3f\n',w,ep);
fprintf('PART b: unit step u=1 for t>=0; normalized angular-acceleration input.\n');
for k=1:6
 [~,Z{k}]=ode45(@(~,z)[z(2);U-10*sin(z(1))],t,IC(k,:)',opts);
 [~,Zfree{k}]=ode45(@(~,z)[z(2);-10*sin(z(1))],t,IC(k,:)',opts);
 e=.5*Z{k}(:,2).^2+10*(1-cos(Z{k}(:,1)));
 effective=e-U*Z{k}(:,1); errors(k)=max(abs(effective-effective(1)));
end
assert(max(errors)<1e-6,'Step-response energy/work balance failed');
f=newfig('01 | Problem 1 - model and assumptions');
ax=subplot(1,2,1,'Parent',f); hold(ax,'on'); axis(ax,'equal'); axis(ax,[-1.5 1.5 -1.5 .7]); axis(ax,'off');
a=pi/5; b=[sin(a),-cos(a)];
plot(ax,[0 0],[0 -1.2],'--','Color',[.6 .6 .6]);
plot(ax,[0 b(1)],[0 b(2)],'-','LineWidth',6,'Color',colors(1,:));
plot(ax,0,0,'ko','MarkerFaceColor','k'); plot(ax,b(1),b(2),'o','MarkerSize',25,'MarkerFaceColor',colors(1,:));
quiver(ax,b(1),b(2),0,-.4,0,'r','LineWidth',2,'MaxHeadSize',.5);
quiver(ax,b(1),b(2),-.4*sin(a),.4*cos(a),0,'Color',[0 .5 .2],'LineWidth',2,'MaxHeadSize',.5);
ang=linspace(0,a,30); plot(ax,.35*sin(ang),-.35*cos(ang),'k');
text(ax,.12,-.42,'\theta','FontSize',18); text(ax,b(1)+.1,b(2)-.3,'mg','Color','r','FontSize',15);
text(ax,b(1)-.3,b(2)+.4,'rod reaction','Color',[0 .5 .2]);
title(ax,'Fixed pivot, massless rod; downward angle reference');
subplot(1,2,2,'Parent',f); axis off
text(0,.95,{'PROBLEM 1 - nonlinear pendulum', ...
 'm l^2 theta'''' + m g l sin(theta) = torque', ...
 'u = torque/(m l^2), g/l = 10', ...
 'x1 = theta, x2 = theta dot = output y', ...
 'x1 dot = x2', 'x2 dot = u - 10 sin(x1)', ...
 'Part (b): u=1, following your instruction', ...
 'Part (c): pulse areas J=0.1,...,1 rad/s', ...
 'epsilon = 0.05 rad; mass cancels from normalized model', ...
 'Original equilibrium question: also show u=0', ...
 'Undamped: oscillations do not decay by themselves'}, ...
 'VerticalAlignment','top','FontSize',13,'Interpreter','none');
stage(f,out,'01_model');

f=newfig('02 | State-space block diagram'); axis off
annotation(f,'textbox',[.06 .82 .88 .1],'String','u -> SUM -> integral -> x2 = y -> integral -> x1 = theta', ...
 'FontSize',19,'EdgeColor','none','HorizontalAlignment','center','Interpreter','none');
boxes={[.06 .48 .12 .12],'u(t)';[.28 .48 .12 .12],'SUM (+,-)'; ...
 [.49 .48 .13 .12],'1/s : x2';[.74 .48 .13 .12],'1/s : x1'; ...
 [.49 .19 .23 .12],'10 sin(x1)'};
for k=1:size(boxes,1),annotation(f,'textbox',boxes{k,1},'String',boxes{k,2},'FontSize',16,'HorizontalAlignment','center','VerticalAlignment','middle','BackgroundColor',[.90 .95 1]);end
annotation(f,'arrow',[.18 .28],[.54 .54]); annotation(f,'arrow',[.40 .49],[.54 .54]);
annotation(f,'arrow',[.62 .74],[.54 .54]); annotation(f,'arrow',[.87 .93],[.54 .54]);
annotation(f,'line',[.90 .90],[.54 .25]);annotation(f,'arrow',[.90 .72],[.25 .25]);
annotation(f,'line',[.49 .34],[.25 .25]); annotation(f,'arrow',[.34 .34],[.25 .48]);
annotation(f,'textbox',[.06 .02 .88 .1],'String','Integrator ICs: first = x2(0); second = x1(0). Feedback enters the MINUS port.', ...
 'EdgeColor','none','FontSize',14,'HorizontalAlignment','center');
stage(f,out,'02_block_diagram');

f=newfig('03 | Unit-step time responses - all six initial conditions');
tl=tiledlayout(f,3,2,'TileSpacing','compact');
for k=1:6
 ax=nexttile(tl); yyaxis(ax,'left'); plot(ax,t,Z{k}(:,1),'Color',colors(k,:),'LineWidth',1.5);ylabel(ax,'theta (rad)');
 yyaxis(ax,'right');plot(ax,t,Z{k}(:,2),'Color',[.25 .25 .25],'LineWidth',1);ylabel(ax,'y = theta dot (rad/s)');
 title(ax,names{k},'Interpreter','none');xlabel(ax,'Time (s)');grid(ax,'on');
end
title(tl,'Part (b): normalized unit step u=1 | colored: angle, dark: output y');
stage(f,out,'03_step_responses');

f=newfig('04 | Phase planes - unit step and original unforced system');
tl=tiledlayout(f,1,2,'TileSpacing','compact');
for panel=1:2
 ax=nexttile(tl);hold(ax,'on');[xx,vv]=meshgrid(linspace(-pi,3*pi,30),linspace(-8,8,23));
 u=double(panel==1); aa=u-10*sin(xx);mag=sqrt(vv.^2+aa.^2);mag(mag==0)=1;
 quiver(ax,xx,vv,vv./mag,aa./mag,.45,'Color',[.82 .82 .82]);
 if panel==1, zz=Z;else, zz=Zfree;end
 for k=1:6, plot(ax,zz{k}(:,1),zz{k}(:,2),'LineWidth',1.6,'Color',colors(k,:));plot(ax,IC(k,1),IC(k,2),'o','Color',colors(k,:),'MarkerFaceColor',colors(k,:));end
 alpha=asin(u/10); stable=alpha+(-1:2)*2*pi; unstable=pi-alpha+(-1:2)*2*pi;
 plot(ax,stable,zeros(size(stable)),'go','MarkerFaceColor',[0 .6 .2],'MarkerSize',8);
 plot(ax,unstable,zeros(size(unstable)),'rx','LineWidth',2,'MarkerSize',10);
 axis(ax,[-pi 3*pi -8 8]);xlabel(ax,'theta (rad, unwrapped)');ylabel(ax,'theta dot (rad/s)');grid(ax,'on');
 if panel==1,title(ax,'Unit step u=1: shifted equilibria');else,title(ax,'Original system u=0: centers and saddles');end
end
title(tl,'Part (d): initial states = colored dots | centers = green | saddles = red crosses');
stage(f,out,'04_phase_planes');

f=newfig('05 | Pulse and impulse responses');tl=tiledlayout(f,2,2,'TileSpacing','compact');
J=.1:.1:1; width=.02; period=4; tp=(0:.005:12)'; pulseZ=cell(10,1); pulseColors=parula(10);
ax1=nexttile(tl);hold(ax1,'on'); ax2=nexttile(tl);hold(ax2,'on');
ax3=nexttile(tl);hold(ax3,'on'); ax4=nexttile(tl);hold(ax4,'on');
for k=1:10
 % One ideal normalized impulse at t=0: velocity jumps by its area J.
 [~,zi]=ode45(@(~,z)[z(2);-10*sin(z(1))],tp,[0;J(k)],opts);
 plot(ax1,tp,zi(:,2),'Color',pulseColors(k,:));
 % Repeated finite pulses, integrated separately across EVERY switch.
 zstart=[0;0]; pp=zeros(numel(tp),2); edges=unique([0:period:12,width:period:12,12]);
 for s=1:numel(edges)-1
  ta=edges(s);tb=edges(s+1);amp=(mod((ta+tb)/2,period)<width)*J(k)/width;
  sol=ode45(@(~,z)[z(2);amp-10*sin(z(1))],[ta tb],zstart,opts);
  mask=tp>=ta & tp<=tb; pp(mask,:)=deval(sol,tp(mask))';zstart=deval(sol,tb);
 end
 pulseZ{k}=pp;plot(ax2,tp,pp(:,2),'Color',pulseColors(k,:));
 plot(ax3,pp(:,1),pp(:,2),'Color',pulseColors(k,:));
 plot(ax4,tp,pp(:,1),'Color',pulseColors(k,:));
end
title(ax1,'Single ideal impulse: output y');xlabel(ax1,'Time (s)');ylabel(ax1,'rad/s');
title(ax2,'Repeated pulses: output y');xlabel(ax2,'Time (s)');ylabel(ax2,'rad/s');
title(ax3,'Repeated pulses: phase plane');xlabel(ax3,'theta (rad)');ylabel(ax3,'theta dot (rad/s)');
title(ax4,'Repeated pulses: angle');xlabel(ax4,'Time (s)');ylabel(ax4,'theta (rad)');
for ax=[ax1 ax2 ax3 ax4],grid(ax,'on');end
legend(ax1,compose('J=%.1f',J),'Location','eastoutside');
title(tl,'Part (c): pulse AREA J=0.1:0.1:1 | width=0.02 s | period=4 s | amplitude=J/width');
stage(f,out,'05_pulse_responses');

f=newfig('06 | Energy, work and equilibrium classification');tl=tiledlayout(f,2,2,'TileSpacing','compact');
ax=nexttile(tl);hold(ax,'on');
for k=1:6,e=.5*Z{k}(:,2).^2+10*(1-cos(Z{k}(:,1)));plot(ax,t,e,'Color',colors(k,:));end
title(ax,'Mechanical energy changes under applied step');ylabel(ax,'E/(m l^2)');xlabel(ax,'Time (s)');grid(ax,'on');
ax=nexttile(tl);hold(ax,'on');
for k=1:6,e=.5*Z{k}(:,2).^2+10*(1-cos(Z{k}(:,1)));balance=e-e(1)-U*(Z{k}(:,1)-Z{k}(1,1));plot(ax,t,balance,'Color',colors(k,:));end
title(ax,'Check: E(t)-E(0)-u[theta(t)-theta(0)] = 0');xlabel(ax,'Time (s)');ylabel(ax,'Energy/work residual');grid(ax,'on');
ax=nexttile(tl);q=linspace(-pi,3*pi,600);plot(ax,q,10*(1-cos(q)),'LineWidth',2);hold(ax,'on');plot(ax,[0 2*pi],[0 0],'go','MarkerFaceColor','g');plot(ax,[pi 3*pi],[20 20],'rx','LineWidth',2);xlabel(ax,'theta');ylabel(ax,'Potential/(m l^2)');title(ax,'u=0: minima = centers, maxima = saddles');grid(ax,'on');
ax=nexttile(tl);axis(ax,'off');
text(ax,0,.95,{'PART (e): EQUILIBRIA', ...
 'Original u=0: (theta, theta dot)=(k*pi, 0)', ...
 'theta=2k*pi: eigenvalues +/- i sqrt(10)', ...
 'Stable centers; NOT asymptotically stable', ...
 'theta=(2k+1)*pi: eigenvalues +/- sqrt(10)', ...
 'Unstable saddles', ...
 'Step u=1: alpha=asin(0.1)=0.100167 rad', ...
 'Centers: alpha+2k*pi; saddles: pi-alpha+2k*pi', ...
 'Case VI stays still only for u=0; step excites it'}, ...
 'VerticalAlignment','top','FontSize',11,'Interpreter','none');
stage(f,out,'06_energy_equilibria');

save(fullfile(out,'problem_1_data.mat'),'t','Z','Zfree','IC','names','J','tp','pulseZ','g','l','ep','U','errors');
writetable(table((1:6)',IC(:,1),IC(:,2),errors,'VariableNames', ...
 {'Case','theta0_rad','velocity0_rad_s','MaxEnergyWorkResidual'}),fullfile(out,'verification.csv'));
build_problem_1_simulink(root);
fprintf('Numerical check PASSED. Maximum energy/work residual: %.3g\n',max(errors));
fprintf('Saved figures/data to: %s\n',out);
f=animate_all(t,Z,names,colors,out,true);
uicontrol(f,'Style','pushbutton','String','REPLAY animation','Units','normalized', ...
 'Position',[.40 .005 .20 .045],'FontSize',12,'Callback',@(~,~)animate_all(t,Z,names,colors,out,false));
fprintf('PROBLEM_1_COMPLETE\n');diary off
end

function f=newfig(name)
f=figure('Name',name,'NumberTitle','off','Theme','dark','Color',[.10 .11 .14],'Position',[70 65 1180 720]);
end
function stage(f,out,name)
style_problem_1(f);drawnow;exportgraphics(f,fullfile(out,[name '.png']),'Resolution',140,'BackgroundColor',f.Color);savefig(f,fullfile(out,[name '.fig']));
fprintf('VISUAL READY: %s\n',name);pause(3);
end
function f=animate_all(t,Z,names,colors,out,record)
f=newfig('07 | LIVE - six pendulums under unit-step input');tl=tiledlayout(f,2,3,'TileSpacing','compact','Padding','compact');
rod=gobjects(6,1);bob=rod;label=rod;trail=rod;
for k=1:6
 ax=nexttile(tl);hold(ax,'on');axis(ax,'equal');axis(ax,[-1.3 1.3 -1.3 1.3]);grid(ax,'on');
 plot(ax,[0 0],[0 -1.1],'--','Color',[.7 .7 .7]);plot(ax,0,0,'ko','MarkerFaceColor','k');
 trail(k)=plot(ax,nan,nan,'-','Color',.65+.35*colors(k,:));
 rod(k)=plot(ax,[0 0],[0 -1],'-','Color',colors(k,:),'LineWidth',5);
 bob(k)=plot(ax,0,-1,'o','Color',colors(k,:),'MarkerFaceColor',colors(k,:),'MarkerSize',19);
 label(k)=text(ax,-1.2,1.1,'','FontSize',10,'Interpreter','none');
 title(ax,names{k},'Interpreter','none');xlabel(ax,'Horizontal position (m)');ylabel(ax,'Vertical position (m)');
end
style_problem_1(f);
if record,video=VideoWriter(fullfile(out,'problem_1_animation.mp4'),'MPEG-4');video.FrameRate=10;open(video);end
indices=1:10:numel(t); clock=tic;
for j=1:numel(indices)
 if ~isgraphics(f),break;end
 i=indices(j);
 for k=1:6
  theta=Z{k}(i,1);bx=sin(theta);by=-cos(theta);
  set(rod(k),'XData',[0 bx],'YData',[0 by]);set(bob(k),'XData',bx,'YData',by);
  hist=max(1,i-70):i;set(trail(k),'XData',sin(Z{k}(hist,1)),'YData',-cos(Z{k}(hist,1)));
  set(label(k),'String',sprintf('$\\theta=%.3f$ rad | $y=\\dot{\\theta}=%.3f$ rad/s',theta,Z{k}(i,2)),'Interpreter','latex');
 end
 title(tl,sprintf('LIVE | t=%.2f s | step u=1 | angle measured from DOWNWARD vertical',t(i)));
 drawnow;
 if record,writeVideo(video,getframe(f));end
 pause(max(0,t(i)-toc(clock)));
end
if record,close(video);exportgraphics(f,fullfile(out,'07_animation.png'),'Resolution',140,'BackgroundColor',f.Color);end
end
