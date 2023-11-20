clear; clc; close all;
% plots heart beat data stream
filename = 'heartbeat-050Hz.xlsx';
%sheet = 1;
timeArray = xlsread(filename,'D:D');
forceArray = xlsread(filename,'J:J');
contactAreaArray = xlsread(filename,'L:L');
beatArray = xlsread(filename,'G:G');
%400/155

%% Initialize video
myVideo = VideoWriter('myVideoFile'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)

total = 155;
for i = 1:total
   clf;

   plot(timeArray(1:i), forceArray(1:i),'LineWidth',3,'Color','w');
   set(gca,'Color','k','FontSize',16)
   xlim([0,4]);
   ylim([2,5]);
   xlabel("Time (s)");
   ylabel("Force (N)")
    
%    plot(timeArray(1:i), contactAreaArray(1:i),'LineWidth',3,'Color','w');
%    set(gca,'Color','k','FontSize',16)
%    xlim([0,4]);
%    ylim([150,300]);
%    xlabel("Time (s)");
%    ylabel("Predicted contact area (mm^2)")

%     windowSize=25;
%     if i<windowSize+1
%         plot(timeArray(1:i), beatArray(1:i),'LineWidth',3,'Color','w');
%     %elseif i>(total-windowSize)
%     %    plot(timeArray(i-windowSize:total), beatArray(i-windowSize:total),'LineWidth',3,'Color','w');
%     else
%         plot(timeArray(i-windowSize:i), beatArray(i-windowSize:i),'LineWidth',3,'Color','w');
%     end;
%    set(gca,'Color','k','FontSize',16)
%    xlim([0,4]);
%    ylim([10,30]);
   %xlabel("Time (s)");
   %ylabel("Force (N)")
    
   
   fig_gcf = gcf;
   fig_gcf.Color = [0 0 0];
   fig_gcf.ToolBar = 'none';
   fig_gcf.Position =([0,300,600,300]);
    
   fig_gca = gca;
   %set(gca,'XTick',[],'YTick',[])
   %fig_gca.XTick = [];
   %fig_gca.YTick = [];
    
    fig_gca.XColor = [1 1 1];%[0 0 0];
    fig_gca.YColor = [1 1 1];%[0 0 0];
    fig_gca.Box = "off";
    
    %gca.XColor = 'w';
    %gca.YColor = 'w';

   %drawnow;
   pause(2.58*10^-3);

   frame = getframe(gcf); %get frame
   writeVideo(myVideo, frame);
end
close(myVideo)


