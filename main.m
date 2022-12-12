%% V1.3 add more anchors（more cellular） ; tag roaming ; communication range  


close all;clear;clc;

rng(1);  % random seeds

%% simulation setting
% basic parameter
c=physconst("lightspeed");
% sigmaD=0.3;     % UWB Range standard deviation
global atom_t   %atom clock time:standard time
global fc       % UWB center frequency
fc=6489.6e6;    % UWB channel 5

% TDMA setting
T0=0;
slot_cnt=1;
slot_num=10;
slot_interval=2.5e-3;
slot_t=T0:slot_interval:slot_num*slot_interval-slot_interval;  % atom clock

% Tx setting
blink_delay=500e-6;
response_delay=500e-6;
sequence_delay=300e-6;

%% Agents initialization
AgentNum=9;
M_Anchor_Index=[1 2];
S_Anchor_Index=[3:8];
Tag_Index=[9];

% tag_pos=(rand(1,2)-0.5)*10;
tag_pos=[5 2];

AgentPos0=[[0 0];[10 0];[-5 5];[5 5];[15 5];[15 -5]; [5 -5]; [-5 -5]; tag_pos]*10;
AgentV0=zeros(AgentNum,2);  % static mode
Agents = Agents_init(AgentNum,M_Anchor_Index,S_Anchor_Index,Tag_Index,AgentPos0,AgentV0);


% slave and master initialization
% determine the slave anchor for each master anchor and their number;allocate master for slave_anchor
% TODO:slave_init()

Agents(1).slave_=[3 4 7 8];
Agents(2).slave_=[4 5 6 7];

% for i=1:length(M_Anchor_Index)    
%     Agents(M_Anchor_Index(i)).slave_=sort(S_Anchor_Index);
% end

% TODO:master init
% for i=1:length(S_Anchor_Index)
%     Agents(S_Anchor_Index(i)).master_=sort(M_Anchor_Index);
% end



%% Channels initialization
GlobalChannels = Channels_init(Agents);

%% debug and plot param


m_rxtime_series=[];
s_rxtime_series=[];


%% process
for i=1:2
    % find m_anchor of this slot
    for j=1:length(M_Anchor_Index)
        if Agents(M_Anchor_Index(j)).slot_num_==i
            m_id=Agents(M_Anchor_Index(j)).id_;
        end
    end

    atom_t=slot_t(i)+Agents(m_id).skewTrue_;   % every slot begins distributively

    % m_anchor transmit blink
    Agents(m_id).tx_msg_=uwb_msg(0,1,i,m_id);
    Agents(m_id).tx_msg_.pos_=Agents(m_id).pTrue_;
    Agents(m_id).tx_msg_.slave_=Agents(m_id).slave_;
    Agents(m_id).tx_delay_=blink_delay;
    Agents(m_id).tx_time_l_=slot_t(i)+blink_delay;
    Agents(m_id).tx_time_a_=atom_t+blink_delay/(1+Agents(m_id).offsetTrue_);
    Agents(m_id)=uwbTx(Agents(m_id),0); % Tx immediately

    GlobalChannels=GlobalChannelsTx(GlobalChannels,Agents(m_id),m_id);

    % s_anchor receive blink and transmit response;tag receive blink and response
    for j=1:AgentNum
        if j==m_id
            continue;
        elseif GlobalChannels(m_id,j).busy_==1  % channel is busy, can be received       
            Agents(j)=uwbRx(Agents(j),GlobalChannels(m_id,j));  % this 1 will be replaced with find busy channel;if more than one busy channel,choose the rx_time smaller one    
            GlobalChannels(m_id,j).busy_=0;
            frame_type=GlobalChannels(m_id,j).uwb_msg_.frame_type_;
            tx_id=GlobalChannels(m_id,j).uwb_msg_.tx_id_;
            slaves=GlobalChannels(m_id,j).uwb_msg_.slave_;      
            if Agents(j).role_==2  % Tag

                Agents(j).m_rxtime_=[Agents(j).m_rxtime_;[tx_id, Agents(j).rx_time_l_]];  % rx_time1
                m_rxtime_series=[m_rxtime_series;[tx_id, Agents(j).rx_time_l_]];
                Agents(j).map_(tx_id,:)=Agents(j).rx_msg_.pos_;
            elseif Agents(j).role_==1   % Slave Anchor       
                if ismember(j,slaves)
                    Agents(j).Deleyed_TX_=1;              
                    Agents(j).tx_delay_=(find(slaves==j)-1)*sequence_delay+response_delay;
                    Agents(j).tx_time_l_=Agents(j).rx_time_l_+Agents(j).tx_delay_;
                    Agents(j).tx_time_a_=Agents(j).rx_time_a_+Agents(j).tx_delay_/(1+Agents(j).offsetTrue_);
                    Agents(j).tx_msg_=uwb_msg(1,1,i,j);
                    Agents(j).tx_msg_.t_reply_=Agents(j).tx_time_l_-Agents(j).rx_time_l_;
                    Agents(j).tx_msg_.pos_=Agents(j).pTrue_;
                    Agents(j).tx_msg_.mt_id_=tx_id;
                end
            end
        end 
    end

    for j=1:4
        s_id=slaves(j);
        Agents(s_id)=uwbTx(Agents(s_id),Agents(s_id).Deleyed_TX_);
        GlobalChannels=GlobalChannelsTx(GlobalChannels,Agents(s_id),s_id);

        for k=1:AgentNum
            if k==s_id
                continue;
            elseif GlobalChannels(s_id,k).busy_==1  % channel is busy, can be received
                Agents(k)=uwbRx(Agents(k),GlobalChannels(s_id,k));  % this 1 will be replaced with find busy channel;if more than one busy channel,choose the rx_time smaller one
                GlobalChannels(s_id,k).busy_=0;
                frame_type=GlobalChannels(s_id,k).uwb_msg_.frame_type_;
                tx_id=GlobalChannels(s_id,k).uwb_msg_.tx_id_;
                mt_id=GlobalChannels(s_id,k).uwb_msg_.mt_id_;
                if Agents(k).role_==2  % Tag        
                    Agents(k).map_(tx_id,:)=Agents(k).rx_msg_.pos_;

                    if ismember(mt_id,Agents(k).m_rxtime_(:,1))
                        m_rxtime=Agents(k).m_rxtime_(find(Agents(k).m_rxtime_(:,1)==mt_id),2);
                        s_rxtime=Agents(k).rx_time_l_;
                        % clock offset estimation
                        cfo=estCFO(Agents(s_id),Agents(k));
                        t_reply=Agents(k).rx_msg_.t_reply_;
                        
                        Agents(k).s_rxtime_=[Agents(k).s_rxtime_;[[mt_id, tx_id],Agents(k).rx_time_l_]];
                        s_rxtime_series=[s_rxtime_series;[mt_id, tx_id],Agents(k).rx_time_l_];
                        Agents(k).cfo_=[Agents(k).cfo_;[tx_id,cfo]];
                        Agents(k).tdoa_=[Agents(k).tdoa_;[[mt_id, tx_id],s_rxtime-m_rxtime-cfo*t_reply]];
                    end
                    % TODO:here we assume each meassage can be received successfully,otherwise we need some examination            
                end
            end
        end

    end

    for t=1:length(Tag_Index)
        if length(Agents(Tag_Index(t)).tdoa_)>=4
            Agents(Tag_Index(t))=Fang_tdoa(Agents(Tag_Index(t)));
        end
    end    
end



%% plot positioning information
figure;
hold on;axis equal;grid on;
axis([-50 200 -50 50]*1.2);

for i=1:length(M_Anchor_Index)
    p1=plot(Agents(M_Anchor_Index(i)).pTrue_(1),Agents(M_Anchor_Index(i)).pTrue_(2),'r*');
    text(Agents(M_Anchor_Index(i)).pTrue_(1)+5,Agents(M_Anchor_Index(i)).pTrue_(2)+5,num2str(Agents(M_Anchor_Index(i)).id_));
end
for i=1:length(S_Anchor_Index)
    p2=plot(Agents(S_Anchor_Index(i)).pTrue_(1),Agents(S_Anchor_Index(i)).pTrue_(2),'b*');
    text(Agents(S_Anchor_Index(i)).pTrue_(1)+5,Agents(S_Anchor_Index(i)).pTrue_(2)+5,num2str(Agents(S_Anchor_Index(i)).id_));
end
for i=1:length(Tag_Index)
    p3=plot(Agents(Tag_Index(i)).pTrue_(1),Agents(Tag_Index(i)).pTrue_(2),'g*');
    text(Agents(Tag_Index(i)).pTrue_(1)+5,Agents(Tag_Index(i)).pTrue_(2)+5,num2str(Agents(Tag_Index(i)).id_));
end
for i=1:length(Tag_Index)
    p4=plot(Agents(Tag_Index(i)).p_(1),Agents(Tag_Index(i)).p_(2),'k*');
end



legend([p1,p2,p3,p4],"Master Anchor","Slave Anchor","Tag_{pTrue}","Tag_p",'Location','southeast');


% %% plot tag_rx_time in one slot
figure;
hold on;
p1=stem(m_rxtime_series(:,2),ones(1,length(m_rxtime_series(:,2))));
p2=stem(s_rxtime_series(:,3),ones(1,length(s_rxtime_series(:,3))));
p3=stem(slot_t(1:3),ones(1,3)*1.25);
for i=1:length(m_rxtime_series(:,2))
    p4=text(m_rxtime_series(i,2),1.07,num2str(m_rxtime_series(i,1)));
end
for i=1:length(s_rxtime_series(:,3))
    p5=text(s_rxtime_series(i,3),1.07,num2str(s_rxtime_series(i,2)));
end
legend([p1 p2 p3],"Tag RX Time(blink)","Tag RX Time(response)","Slot Boundary");
% set(p4,'handlevisibility','off');
% set(p5,'handlevisibility','off');
% legend([p1,p2,p3,p4,p5],{"Tag RX Time(blink)","Tag RX Time(response)","Slot Boundary","",""});

xlim([0 6]*1e-3)
ylim([0 1.8])



function GlobalChannels=GlobalChannelsTx(GlobalChannels,Agent,j)
sigmaD=0.3;
c=physconst("lightspeed");
Crange=1000;  % communication range


AgentNum=size(GlobalChannels,2);
for i=1:AgentNum
    if i==j
        continue;
    else
%         GlobalChannels(j,i).busy_=1; 
        if GlobalChannels(j,i).dGt_<=Crange
            GlobalChannels(j,i).busy_=1;       
        end
        GlobalChannels(j,i).tx_=Agent.tx_time_a_;
        GlobalChannels(j,i).rx_=Agent.tx_time_a_+((GlobalChannels(j,i).dGt_+randn*sigmaD)/c);
        GlobalChannels(j,i).uwb_msg_=Agent.tx_msg_;
    end                 
end
end


function Agent=Fang_tdoa(Agent)

a=length(Agent.tdoa_);
c=physconst("lightspeed");

A=zeros(a,3);
ri1=zeros(a,1);
Ki1=zeros(a,1);
for i=1:a

    % with CFO calibration

%     ri1(i)=c*((Agent.rxtime_buffer_(i+1)-Agent.rxtime_buffer_(1))-Agent.cfo_(i+1)*Agent.t_reply_(i+1))-norm(Agent.pos_buffer_(i+1,:)-Agent.pos_buffer_(1,:));

    ri1(i)=c*Agent.tdoa_(i,3)-norm(Agent.map_(Agent.tdoa_(i,1),:)-Agent.map_(Agent.tdoa_(i,2),:));
    % without CFO calKibration
%     ri1(i)=c*((Agent.rxtime_buffer_(i+1)-Agent.rxtime_buffer_(1))-(response_delay+(i-1)*sequence_delay))-norm(Agent.pos_buffer_(i+1,:)-Agent.pos_buffer_(1,:));
    Ki1(i)=norm(Agent.map_(Agent.tdoa_(i,2),:))^2-norm(Agent.map_(Agent.tdoa_(i,1),:))^2;
    A(i,:)=2*[Agent.map_(Agent.tdoa_(i,2),1)-Agent.map_(Agent.tdoa_(i,1),1),Agent.map_(Agent.tdoa_(i,2),2)-Agent.map_(Agent.tdoa_(i,1),2),ri1(i)];
    
end
b=Ki1-ri1.^2;
temp=(A.'*A)\A.'*b;
Agent.p_=temp(1:2);
temp(1:2)


Agent.m_rxtime_=[];
Agent.s_rxtime_=[];
Agent.tdoa_=[];
Agent.cfo_=[];


end

function cfo=estCFO(Agent1,Agent2)
% Agent1:TX  Agent2:RX
global fc
sigmaF=1000;   % carrier frequency estimation standard deviation
cfo=(fc/(1+Agent1.offsetTrue_)+randn(1)*sigmaF)/(fc/(1+Agent2.offsetTrue_)+randn(1)*sigmaF);
end