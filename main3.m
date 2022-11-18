%% add clock drift

close all;clear;clc;

%% simulation setting
% TDMA setting
rng(1);  % random seeds

T0=0;
slot_cnt=1;
slot_num=10;
slot_interval=2.5e-3;
slot_t=T0:slot_interval:slot_num*slot_interval-slot_interval;

% Tx setting
blink_delay=500e-6;
response_delay=500e-6;
sequence_delay=300e-6;

% Agent setting
AgentNum=10;


c=physconst("lightspeed");
sigmaD=0.1;

%% Agents initialization

% first with 1 Master Anchor,4 Slave Anchor,1 tag
AgentPos0=[[0 0];[5 0];[0 5];[-5 0];[0 -5];[3 3];[0 3];[4 2];[3 1];[-3 5]]*10;

AgentV0=zeros(AgentNum,2);  % static mode
Agents=Agent.empty();
for i=1:AgentNum
    Agents(i)=Agent(i,AgentPos0(i,:),AgentV0(i,:));  
end

% role initialization
M_Anchor_Index=[1];
S_Anchor_Index=[2:5];
Tag_Index=[6:10];
M_Anchor=Agents(M_Anchor_Index);
for i=1:length(M_Anchor)
    M_Anchor(i).role_=0;
    Agents(M_Anchor_Index(i)).role_=0;
end
S_Anchor=Agents(S_Anchor_Index);
for i=1:length(S_Anchor)
    S_Anchor(i).role_=1;
    Agents(S_Anchor_Index(i)).role_=1;
end
Tag=Agents(Tag_Index);
for i=1:length(Tag)
    Tag(i).role_=2;
    Agents(Tag_Index(i)).role_=2;
end

% slot initialization (only for M_Anchor)
for i=1:length(M_Anchor_Index)
    M_Anchor(i).slot_num_=i; % slot allocation
    Agents(M_Anchor_Index(i)).slot_num_=i;

    % determine the slave anchor for each master anchor and their number
    M_Anchor(i).slave_=sort(S_Anchor_Index);  %TODO: the algorithm to choose slave anchor
    Agents(M_Anchor_Index(i)).slave_=sort(S_Anchor_Index);
  
end

for i=1:length(S_Anchor_Index)
    Agents(S_Anchor_Index(i)).master_=sort(M_Anchor_Index);
end


GlobalChannels=Channels.empty();
for i=1:AgentNum
    for j=1:AgentNum
        GlobalChannels(i,j)=Channels(i,j,norm(AgentPos0(i,:)-AgentPos0(j,:)));
    end
end


%% debug and plot param




%% process
for i=1:1
    %% Anchor transmit blink
    for j=1:length(M_Anchor)
        if M_Anchor(j).slot_num_==i
            m_id=M_Anchor(j).id_;
        end

    end

    Agents(m_id).tx_msg_=uwb_msg(0,1,i);
    Agents(m_id).tx_msg_.pos_=[Agents(m_id).pTrue_];
    Agents(m_id).tx_msg_.slave_=Agents(m_id).slave_;
    Agents(m_id).tx_delay_=blink_delay;
    Agents(m_id).tx_time_=slot_t(i)+blink_delay;
    Agents(m_id)=uwbTx(Agents(m_id),0); % Tx immediately

    GlobalChannels=GlobalChannelsTx(GlobalChannels,Agents(m_id),m_id);

    %% Slave Anchors receive blink and transmit response
    %% Tags receive blink and response;

    
    
    for j=1:AgentNum
        if j==m_id
            continue;
        elseif GlobalChannels(m_id,j).busy_==1  % channel is busy, can be received

            
            Agents(j)=uwbRx(Agents(j),GlobalChannels(m_id,j));  % this 1 will be replaced with find busy channel;if more than one busy channel,choose the rx_time smaller one
            GlobalChannels(m_id,j).busy_=0;
            slaves=GlobalChannels(m_id,j).uwb_msg_.slave_;
            
            if Agents(j).role_==2  % Tag

                Agents(j).rxtime_buffer_(1)=Agents(j).rx_time_;
                Agents(j).pos_buffer_(1,:)=Agents(j).rx_msg_.pos_;
                % clock offset estimation
                % rx_time1
            
            elseif Agents(j).role_==1   % Slave Anchor
                
                if ismember(j,slaves)
                    Agents(j).Deleyed_TX_=1;
                    Agents(j).tx_delay_=(find(slaves==j)-1)*sequence_delay+response_delay;
                    Agents(j).tx_time_=Agents(j).rx_time_+Agents(j).tx_delay_;
                end
            end
        end 
    end

    for j=1:4
        r=slaves(j);   
        Agents(r)=uwbTx(Agents(r),Agents(r).Deleyed_TX_);
        Agents(r).tx_msg_=uwb_msg(1,1,i);
        Agents(r).tx_msg_.t_reply_=Agents(r).tx_time_-Agents(r).rx_time_;
        Agents(r).tx_msg_.pos_=[Agents(r).pTrue_];        
        GlobalChannels=GlobalChannelsTx(GlobalChannels,Agents(r),r);

        for k=1:AgentNum
            if k==r
                continue;
            elseif GlobalChannels(r,k).busy_==1  % channel is busy, can be received
                Agents(k)=uwbRx(Agents(k),GlobalChannels(r,k));  % this 1 will be replaced with find busy channel;if more than one busy channel,choose the rx_time smaller one
                GlobalChannels(r,k).busy_=0;        
                if Agents(k).role_==2  % Tag
                    
                    Agents(k).rxtime_buffer_(j+1)=Agents(k).rx_time_;
                    Agents(k).pos_buffer_(j+1,:)=Agents(k).rx_msg_.pos_;
                    % TODO:here we assume each meassage can be received successfully,otherwise we need some examination            
                end
            end
        end

    end

    for t=1:length(Tag_Index)
        Agents(Tag_Index(t))=Fang_tdoa(Agents(Tag_Index(t)));
        Agents(Tag_Index(t)).p_
    end




end

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


skew=zeros(10,1);
for i=1:10
    skew(i)=Agents(i).skewTrue_;
end
skew


figure;
hold on;
stem(Agents(6).rxtime_buffer_(1:5),ones(1,5));
stem(slot_t(1:2),ones(1,2)*1.25,'k');
xlim([0 3]*1e-3)
ylim([0 1.8])
legend("Tag RX Time","Slot Boundary");


function GlobalChannels=GlobalChannelsTx(GlobalChannels,Agent,j)

sigmaD=0.1;
c=physconst("lightspeed");

AgentNum=size(GlobalChannels,2);
for i=1:AgentNum
    if i==j
        continue;
    else
        GlobalChannels(j,i).busy_=1;     % TODO:add cover range later
        GlobalChannels(j,i).tx_=Agent.tx_time_;
        GlobalChannels(j,i).rx_=Agent.tx_time_+((GlobalChannels(j,i).dGt_+randn*sigmaD)/c);  %TODO: Add noise later      
        GlobalChannels(j,i).uwb_msg_=Agent.tx_msg_;
    end                 
end
end


function Agent=Fang_tdoa(Agent)
c=physconst("lightspeed");
response_delay=500e-6;
sequence_delay=300e-6;

A=zeros(4,3);
b=zeros(4,1);
ri1=zeros(4,1);
Ki1=zeros(4,1);
for i=1:4
    ri1(i)=c*(Agent.rxtime_buffer_(i+1)-Agent.rxtime_buffer_(1)-response_delay-(i-1)*sequence_delay)-norm(Agent.pos_buffer_(i+1,:)-Agent.pos_buffer_(1,:));
    Ki1(i)=norm(Agent.pos_buffer_(i+1,:))^2-norm(Agent.pos_buffer_(1,:))^2;

    A(i,:)=2*[Agent.pos_buffer_(i+1,1)-Agent.pos_buffer_(1,1),Agent.pos_buffer_(i+1,2)-Agent.pos_buffer_(1,2),ri1(i)];
    
end
b=Ki1-ri1.^2;
temp=(A.'*A)\A.'*b;
Agent.p_=temp(1:2);


end

