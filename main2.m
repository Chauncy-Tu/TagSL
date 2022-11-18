close all;clear all;clc;

%% simulation setting
% TDMA setting
T0=0;
slot_cnt=1;
slot_num=10;
slot_interval=5e-3;
slot_t=T0:slot_interval:slot_num*slot_interval-slot_interval;
% Tx setting
blink_delay=1e-3;
response_delay=1e-3;
% Agent setting
AgentNum=10;
c=physconst("lightspeed");

sigmaD=0.1;

%% Agents initialization
AgentPos0=[[0 0];[10 0];[5 5];[3 0];[3 0];[3 0];[3 0];[3 0];[3 0];[3 0]];
AgentV0=zeros(AgentNum,2);  % static mode
Agents=MyAgent.empty();
for i=1:AgentNum
    Agents(i)=MyAgent(i,AgentPos0(i,:),AgentV0(i,:));  
end





Anchor=Agents(1);
Anchor.role_=0;


for j=2:AgentNum
    Agents(j).slot_num_=j;
end
% Tag=Agents(2);


GlobalChannels=Channels.empty();
for i=1:AgentNum
    for j=1:AgentNum
        GlobalChannels(i,j)=Channels(i,j,norm(AgentPos0(i,:)-AgentPos0(j,:)));
    end
end


t_reply=0;
dist=zeros(AgentNum,1);

Anchor_tx_time=zeros(30,1);
Tag_tx_time=zeros(30,1);

anchor_count=0;
tag_count=0;

for i=1:10
    %% Anchor transmit blink
    Anchor.tx_msg_=uwb_msg(0,1,i);
    Anchor=uwbTx(Anchor,slot_t(i),blink_delay);

    anchor_count=anchor_count+1;
    Anchor_tx_time(anchor_count)=Anchor.tx_time_;

    for j=2:AgentNum
        GlobalChannels(1,j).busy_=1;     % TODO:add cover range later
        GlobalChannels(1,j).tx_=Anchor.tx_time_;
        GlobalChannels(1,j).rx_=Anchor.tx_time_+((GlobalChannels(1,j).dGt_+randn*sigmaD)/c);  %TODO: Add noise later      
        GlobalChannels(1,j).uwb_msg_=Anchor.tx_msg_;
    end

    %% Tag receive blink and transmit response
    for j=2:AgentNum
        Agents(j)=uwbRx(Agents(j),GlobalChannels(1,j));  % this 1 will be replaced with find busy channel;if more than one busy channel,choose the rx_time smaller one
        if Agents(j).role_==1 && Agents(j).slot_num_==i
            Agents(j)=uwbTx(Agents(j),Agents(j).rx_time_,response_delay);
            Agents(j).tx_msg_=uwb_msg(1,1,i);
            Agents(j).tx_msg_.t_reply_=Agents(j).tx_time_-Agents(j).rx_time_;
                    
            GlobalChannels(j,1).busy_=1;     % TODO:add cover range later
            GlobalChannels(j,1).tx_=Agents(j).tx_time_;
            GlobalChannels(j,1).rx_=Agents(j).tx_time_+((GlobalChannels(j,1).dGt_+randn*sigmaD)/c);  %TODO: Add noise later      
            GlobalChannels(j,1).uwb_msg_=Agents(j).tx_msg_;

            tag_count=tag_count+1;
            Tag_tx_time(tag_count)=Agents(j).tx_time_;

        end
        GlobalChannels(1,j).busy_=0;
    end

    %% Anchor receive response and transmit blink
    for j=2:AgentNum
        if GlobalChannels(j,1).busy_==1
            Anchor=uwbRx(Anchor,GlobalChannels(j,1));
            
            
            Anchor=uwbTx(Anchor,Anchor.rx_time_,blink_delay);
            Anchor.tx_msg_=uwb_msg(0,1,i);
            Anchor.tx_msg_.t_reply_=Anchor.tx_time_-Agents(j).rx_time_;

            anchor_count=anchor_count+1;
            Anchor_tx_time(anchor_count)=Anchor.tx_time_;

                    
            GlobalChannels(1,j).busy_=1;     % TODO:add cover range later
            GlobalChannels(1,j).tx_=Anchor.tx_time_;
            GlobalChannels(1,j).rx_=Anchor.tx_time_+((GlobalChannels(1,j).dGt_+randn*sigmaD)/c);  %TODO: Add noise later      
            GlobalChannels(1,j).uwb_msg_=Anchor.tx_msg_;


            T_round=Anchor.rx_time_-Anchor.tx_time_;
            T_reply=Anchor.rx_msg_.t_reply_;
            tof=(T_round-T_reply)/2;
            dist(j)=tof*c;


            GlobalChannels(j,1).busy_=0;
        end
    end

    %% Tag receive blink and transmit response2 
    for j=2:AgentNum
        Agents(j)=uwbRx(Agents(j),GlobalChannels(1,j));  % this 1 will be replaced with find busy channel;if more than one busy channel,choose the rx_time smaller one
        if Agents(j).role_==1 && Agents(j).slot_num_==i
            Agents(j)=uwbTx(Agents(j),Agents(j).rx_time_,response_delay);
            Agents(j).tx_msg_=uwb_msg(1,1,i);
            Agents(j).tx_msg_.t_reply_=Agents(j).tx_time_-Agents(j).rx_time_;
                    
            GlobalChannels(j,1).busy_=1;     % TODO:add cover range later
            GlobalChannels(j,1).tx_=Agents(j).tx_time_;
            GlobalChannels(j,1).rx_=Agents(j).tx_time_+((GlobalChannels(j,1).dGt_+randn*sigmaD)/c);  %TODO: Add noise later      
            GlobalChannels(j,1).uwb_msg_=Agents(j).tx_msg_;

            tag_count=tag_count+1;
            Tag_tx_time(tag_count)=Agents(j).tx_time_;

        end
        GlobalChannels(1,j).busy_=0;
    end
    
    %% Anchor receive response2
     for j=2:AgentNum
        if GlobalChannels(j,1).busy_==1
            Anchor=uwbRx(Anchor,GlobalChannels(j,1));
            T_round=Anchor.rx_time_-Anchor.tx_time_;
            T_reply=Anchor.rx_msg_.t_reply_;
            tof=(T_round-T_reply)/2;
            dist(j)=tof*c;


            GlobalChannels(j,1).busy_=0;
        end
    end



end

figure;
hold on;
stem(Anchor_tx_time(1:anchor_count),ones(1,anchor_count),'b');
stem(Tag_tx_time(1:tag_count),ones(1,tag_count),'r');
stem(slot_t,ones(1,slot_num)*1.25,'k');
ylim([0 1.8])
legend("Anchor TX","Tag TX","Slot Boundary");

distGT=zeros(10,1);
for i=1:10
    distGT(i)=GlobalChannels(1,i).dGt_;
end

figure;
hold on;
plot(1:10,abs(distGT-dist));
legend("range error");
xlabel("Agent num");
ylabel("/m");
