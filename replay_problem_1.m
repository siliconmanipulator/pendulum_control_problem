function f=replay_problem_1(record)
if nargin<1,record=false;end
root=fileparts(mfilename('fullpath'));out=fullfile(root,'outputs');
d=load(fullfile(out,'problem_1_data.mat'));t=d.t;Z=d.Z;names=d.names;colors=lines(6);
f=figure('Name','07 | LIVE - six pendulums under unit-step input','NumberTitle','off','Theme','dark','Color',[.10 .11 .14],'Position',[70 65 1180 720]);tl=tiledlayout(f,2,3,'TileSpacing','compact','Padding','compact');
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
uicontrol(f,'Style','pushbutton','String','REPLAY animation','Units','normalized','Position',[.40 .005 .20 .045],'FontSize',12,'Callback',@(~,~)replay_problem_1(false));
if record,close(video);exportgraphics(f,fullfile(out,'07_animation.png'),'Resolution',140,'BackgroundColor',f.Color);end
end
