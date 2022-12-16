close all;clear all;clc;
if length(instrfind) ~= 0
    fclose(instrfind);
end

s = serial('COM15');
s.BaudRate = 115200;
%% record the data
fopen(s);
filename = date;
fid = fopen([filename,'.DAT'], 'wb');

x=[];
y=[];

f1 = figure(1);
set(f1, 'Position', [15 45 1000 500]);
% f1 = figure(2);
% set(f1, 'Position', [15 45 1500 700]);

while true
    tline = fgetl(s);
    
    fprintf(fid, tline);
   
    num = str2double(regexp(tline,' ','split'));
    if length(num)>1
        x=[x;num(2)];
        y=[y;num(3)];
    end
    
    if length(x) > 10
        cnt = 10;
        if (length(x) > 300) 
            cnt = 300;
        else
            cnt = length(x);
        end
      
        figure(1);
        clf;
        axis([0 cnt -1.5 0]);
        hold on;
        h1 = plot(x(end - cnt + 1 : end));
    end
    
end
