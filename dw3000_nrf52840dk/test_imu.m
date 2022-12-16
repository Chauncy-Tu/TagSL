close all;clear all;clc;
if length(instrfind) ~= 0
    fclose(instrfind);
end

s = serial('COM16');
s.BaudRate = 115200;
%% record the data
fopen(s);
filename = date;
fid = fopen([filename,'.DAT'], 'wb');

an=[];
wn=[];
temp_an=[];
temp_wn=[];
var_an=[];
var_wn=[];


% f1 = figure(1);
% set(f1, 'Position', [15 65 484 316]);
% f2 = figure(2);
% set(f2, 'Position', [515 45 484 316]);
% f1 = figure(1);
% set(f1, 'Position', [15 65 700 700]);
% f2 = figure(2);
% set(f2, 'Position', [800 65 700 700]);


% f1 = figure(1);
% set(f1, 'Position', [15 45 484 316]);
% f2 = figure(2);
% set(f2, 'Position', [515 45 484 316]);
f3 = figure(3);
set(f3, 'Position', [15 45 1500 700]);
% f4 = figure(4);
% set(f4, 'Position', [573 463 404 316]);

while true
    tline = fgetl(s);
    
    fprintf(fid, tline);
   
    num = str2double(regexp(tline,' ','split'));
    if length(num)>1
        an=[an;num(2)];
        wn=[wn;num(3)];

        if length(temp_an)<5
            temp_an=[temp_an;num(2)];
            temp_wn=[temp_wn;num(3)];
        else
            temp_an=[temp_an(2:end-1);num(2)];
            temp_wn=[temp_wn(2:end-1);num(3)];
        end
    end
    
    var_an=[var_an;var(temp_an)];
    var_wn=[var_wn;var(temp_wn)];
    
    if length(an) > 10
            cnt = 10;
            if (length(an) > 300) 
                cnt = 300;
            else
                cnt = length(an);
            end
            
            
            
%             figure(1);
%             clf;
%             axis([0 cnt 9 15]);
%             hold on;
%             h1 = plot(an(end - cnt + 1 : end));
% 
%             figure(2);
%             clf;
%             axis([0 cnt 0 50]);
%             hold on;
%             h2 = plot(wn(end - cnt + 1 : end));

            figure(3);
            clf;
            axis([0 cnt 0 4]);
            hold on;
            h2 = plot(var_an(end - cnt + 1 : end));

%             figure(4);
%             clf;
%             axis([0 cnt 0 150]);
%             hold on;
%             h2 = plot(var_wn(end - cnt + 1 : end));

         

        end
    
end
