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

tdoa_cal=[];


% f1 = figure(1);
% set(f1, 'Position', [15 65 484 316]);



f1 = figure(1);
set(f1, 'Position', [15 45 1000 500]);
% f1 = figure(2);
% set(f1, 'Position', [15 45 1500 700]);

while true
    tline = fgetl(s);
    
    fprintf(fid, tline);
   
    num = str2double(regexp(tline,' ','split'));
    if length(num)>1
        tdoa_cal=[tdoa_cal;num(3)];
    end
    
    if length(tdoa_cal) > 10
        cnt = 10;
        if (length(tdoa_cal) > 300) 
            cnt = 300;
        else
            cnt = length(tdoa_cal);
        end
      
        figure(1);
        clf;
        axis([0 cnt 300 450]);
        hold on;
        h1 = plot(tdoa_cal(end - cnt + 1 : end));
    end
    
end
