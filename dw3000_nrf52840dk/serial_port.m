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
obj1.InputBufferSize =512;%���뻺����
obj1.OutputBufferSize =512;%���������
obj1.ReadAsyncMode = 'continuous';%��ͬͨ��ģʽ�£���ȡ�������ݲ��������������ݷ�ʽ����λ�����������Զ��������뻺�����С�
obj1.BaudRate  = 115200;%���ò�����
obj1.Parity = 'none';%��У��λ
obj1.StopBits  = 1;%1��ֹͣλ
obj1.DataBits  = 8;%8������λ
obj1.Terminator = 'LF';%������ֹ����CRΪ�س�����LFΪ���з���
obj1.FlowControl  = 'none';%����
obj1.timeout  = 1.0;%һ�β�����ʱʱ��
obj1.BytesAvailableFcnMode =  'byte';%���ݶ����ʽ
obj1.BytesAvailableFcnCount  = 1024;%�����жϵ���������
obj1.BytesAvailableFcn  = @callback;%���ڽ����жϻص�����
fopen(obj1);


% Communicating with instrument object, obj1.
str_recv="";
len=512;                            % ��ȡ����
% ranges=zeros(20,1);
dtu=1/499.2e6/128;

timestamp_prev=0;

rx_time_array=zeros(10,13);
count=1;
while(1)
    data1 = fscanf(obj1,'uint8',len);    % read the data of length len
    data1=strip(data1,"both",char(13));  % strip the newline
    data1=strip(data1,"both",newline);
    str=sprintf("%s",data1);
    str_recv=str_recv+str;
    fprintf("\n%s",str);
    
    temp=split(str,[":"," "]);
    
    
end
save("data/slot_analysis11.mat","rx_time_array");
fclose(obj1);
delete(instrfindall('Type','serial'));