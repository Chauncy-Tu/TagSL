close all;clear all;clc;
%% Instrument Connection
warning off;
% Find a serial port object.
obj1 = instrfind('Type', 'serial', 'Port', 'COM15', 'Tag', '');
% Create the serial port object if it does not exist
% otherwise use the object that was found.
if isempty(obj1)
    obj1 = serial('COM15');
else
    fclose(obj1);
    obj1 = obj1(1);
end

%% Disconnect and Clean Up
fclose(obj1);
%% Instrument Connection 
%Connect to instrument object, obj1
obj1.InputBufferSize =512;%输入缓冲区
obj1.OutputBufferSize =512;%输出缓冲区
obj1.ReadAsyncMode = 'continuous';%异同通信模式下，读取串口数据采用连续接收数据方式，下位机返回数据自动存入输入缓冲区中。
obj1.BaudRate  = 115200;%设置波特率
obj1.Parity = 'none';%无校验位
obj1.StopBits  = 1;%1个停止位
obj1.DataBits  = 8;%8个数据位
obj1.Terminator = 'LF';%设置终止符（CR为回车符，LF为换行符）
obj1.FlowControl  = 'none';%流控
obj1.timeout  = 1.0;%一次操作超时时间
obj1.BytesAvailableFcnMode =  'byte';%数据读入格式
obj1.BytesAvailableFcnCount  = 1024;%触发中断的数据数量
obj1.BytesAvailableFcn  = @callback;%串口接收中断回调函数
fopen(obj1);


% Communicating with instrument object, obj1.
str_recv="";
len=512;                            % 读取长度
% ranges=zeros(20,1);
dtu=1/499.2e6/128;

timestamp_prev=0;

rx_time_array=zeros(10,13);
count=1;
tdoa_cal=[];
while(1)
    data1 = fscanf(obj1,'uint8',len);    % read the data of length len
    data1=strip(data1,"both",char(13));  % strip the newline
    data1=strip(data1,"both",newline);
    str=sprintf("%s",data1);
    str_recv=str_recv+str;
    fprintf("\n%s",str);
    
    temp=split(str,[" "]);
    if(length(temp)>1)
        tdoa_cal=[tdoa_cal;str2num(temp(3))];
    end
   
end
%save("data/slot_analysis11.mat","rx_time_array");
fclose(obj1);
delete(instrfindall('Type','serial'));