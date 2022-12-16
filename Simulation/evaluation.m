close all;clear;clc;

rng(1);  % random seeds

%% simulation setting
% basic parameter
c=physconst("lightspeed");
sigmaD=0.3;     % UWB Range standard deviation
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
AgentNum=6;
M_Anchor_Index=[1];
S_Anchor_Index=[2:5];
Tag_Index=[6];

exp_num=100;
TagPos_True=zeros(exp_num,2);
TagPos_Est=zeros(exp_num,2);
Pos_err=zeros(exp_num,2);
err=zeros(exp_num,1);

for iter=1:exp_num

    tag_pos=(rand(1,2)-0.5)*100;

    TagPos_True(iter,:)=tag_pos;

    AgentPos0=[[0 0];[50 50];[-50 50];[-50 -50];[50 -50];tag_pos];
    AgentV0=zeros(AgentNum,2);  % static mode
    Agents = Agents_init(AgentNum,M_Anchor_Index,S_Anchor_Index,Tag_Index,AgentPos0,AgentV0);
    
    %% Channels initialization
    GlobalChannels = Channels_init(Agents);
    
    %% debug and plot param
    
    
    %% process
    for i=1:1
        % find m_anchor of this slot
        for j=1:length(M_Anchor_Index)
            if Agents(M_Anchor_Index(j)).slot_num_==i
                m_id=Agents(M_Anchor_Index(j)).id_;
            end
        end
    
        atom_t=slot_t(i)+Agents(m_id).skewTrue_;   % every slot begins distributively
    
        % m_anchor transmit blink
        Agents(m_id).tx_msg_=uwb_msg(0,1,i);
        Agents(m_id).tx_msg_.pos_=[Agents(m_id).pTrue_];
        Agents(m_id).tx_msg_.slave_=Agents(m_id).slave_;
        Agents(m_id).tx_delay_=blink_delay;
        Agents(m_id).tx_time_l_=slot_t(i)+blink_delay;
        Agents(m_id).tx_time_a_=atom_t+blink_delay/(1+Agents(m_id).offsetTrue_);
        Agents(m_id)=uwbTx(Agents(m_id),0); % Tx immediately
    
        GlobalChannels=GlobalChannelsTx(GlobalChannels,Agents(m_id),m_id);
    
        % s_anchor receive blink and transmit response
        % tag receive blink and response
        
        for j=1:AgentNum
            if j==m_id
                continue;
            elseif GlobalChannels(m_id,j).busy_==1  % channel is busy, can be received       
                Agents(j)=uwbRx(Agents(j),GlobalChannels(m_id,j));  % this 1 will be replaced with find busy channel;if more than one busy channel,choose the rx_time smaller one    
                GlobalChannels(m_id,j).busy_=0;
                slaves=GlobalChannels(m_id,j).uwb_msg_.slave_;      
                if Agents(j).role_==2  % Tag
                    Agents(j).rxtime_buffer_(1)=Agents(j).rx_time_l_;  % rx_time1
                    Agents(j).pos_buffer_(1,:)=Agents(j).rx_msg_.pos_;
                    % clock offset estimation
                    Agents(j).cfo_(1)=estCFO(Agents(m_id),Agents(j));
                elseif Agents(j).role_==1   % Slave Anchor       
                    if ismember(j,slaves)
                        Agents(j).Deleyed_TX_=1;              
                        Agents(j).tx_delay_=(find(slaves==j)-1)*sequence_delay+response_delay;
                        Agents(j).tx_time_l_=Agents(j).rx_time_l_+Agents(j).tx_delay_;
                        Agents(j).tx_time_a_=Agents(j).rx_time_a_+Agents(j).tx_delay_/(1+Agents(j).offsetTrue_);
                        Agents(j).tx_msg_=uwb_msg(1,1,i);
                        Agents(j).tx_msg_.t_reply_=Agents(j).tx_time_l_-Agents(j).rx_time_l_;
                        Agents(j).tx_msg_.pos_=[Agents(j).pTrue_];
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
                    if Agents(k).role_==2  % Tag
                        
                        Agents(k).rxtime_buffer_(j+1)=Agents(k).rx_time_l_;
                        Agents(k).pos_buffer_(j+1,:)=Agents(k).rx_msg_.pos_;
                        % clock offset estimation
                        Agents(k).cfo_(j+1)=estCFO(Agents(s_id),Agents(k));
                        Agents(k).t_reply_(j+1)=Agents(k).rx_msg_.t_reply_;
                        % TODO:here we assume each meassage can be received successfully,otherwise we need some examination            
                    end
                end
            end
    
        end
    
        for t=1:length(Tag_Index)
            Agents(Tag_Index(t))=Fang_tdoa(Agents(Tag_Index(t)));
            Agents(Tag_Index(t)).p_;
        end    
    end

    TagPos_Est(iter,:)=Agents(Tag_Index(t)).p_;
    Pos_err(iter,:)=TagPos_True(iter,:)-TagPos_Est(iter,:);
    err(iter,:)=norm(Pos_err(iter,:));

end

mean_error=mean(err)





%% plot positioning information
figure;
hold on;axis equal;grid on;
axis([-50 50 -50 50]*1.2);

for i=1:length(M_Anchor_Index)
    p1=plot(Agents(M_Anchor_Index(i)).pTrue_(1),Agents(M_Anchor_Index(i)).pTrue_(2),'r*');
end
for i=1:length(S_Anchor_Index)
    p2=plot(Agents(S_Anchor_Index(i)).pTrue_(1),Agents(S_Anchor_Index(i)).pTrue_(2),'b*');
end
for i=1:length(Tag_Index)
    p3=plot(Agents(Tag_Index(i)).pTrue_(1),Agents(Tag_Index(i)).pTrue_(2),'g*');
    
end
for i=1:length(Tag_Index)
    p4=plot(Agents(Tag_Index(i)).p_(1),Agents(Tag_Index(i)).p_(2),'k*');
    
end
legend([p1,p2,p3,p4],"Master Anchor","Slave Anchor","Tag_{pTrue}","Tag_p",'Location','southeast');


%% plot tag_rx_time in one slot
figure;
hold on;
stem(Agents(6).rxtime_buffer_(1:5),ones(1,5));
stem(slot_t(1:2),ones(1,2)*1.25,'k');
xlim([0 3]*1e-3)
ylim([0 1.8])
legend("Tag RX Time","Slot Boundary");


function GlobalChannels=GlobalChannelsTx(GlobalChannels,Agent,j)
sigmaD=0.3;
c=physconst("lightspeed");
Crange=300;  % communication range


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
c=physconst("lightspeed");
response_delay=500e-6;
sequence_delay=300e-6;

A=zeros(4,3);
ri1=zeros(4,1);
Ki1=zeros(4,1);
for i=1:4

    % with CFO calibration
    ri1(i)=c*((Agent.rxtime_buffer_(i+1)-Agent.rxtime_buffer_(1))-Agent.cfo_(i+1)*Agent.t_reply_(i+1))-norm(Agent.pos_buffer_(i+1,:)-Agent.pos_buffer_(1,:));
    % without CFO calibration
%     ri1(i)=c*((Agent.rxtime_buffer_(i+1)-Agent.rxtime_buffer_(1))-(response_delay+(i-1)*sequence_delay))-norm(Agent.pos_buffer_(i+1,:)-Agent.pos_buffer_(1,:));
    Ki1(i)=norm(Agent.pos_buffer_(i+1,:))^2-norm(Agent.pos_buffer_(1,:))^2;
    A(i,:)=2*[Agent.pos_buffer_(i+1,1)-Agent.pos_buffer_(1,1),Agent.pos_buffer_(i+1,2)-Agent.pos_buffer_(1,2),ri1(i)];
    
end
b=Ki1-ri1.^2;
temp=(A.'*A)\A.'*b;
Agent.p_=temp(1:2);

end

function cfo=estCFO(Agent1,Agent2)
% Agent1:TX  Agent2:RX
global fc
sigmaF=1000;   % carrier frequency estimation standard deviation
cfo=(fc/(1+Agent1.offsetTrue_)+randn(1)*sigmaF)/(fc/(1+Agent2.offsetTrue_)+randn(1)*sigmaF);
end