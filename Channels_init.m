function GlobalChannels = Channels_init(Agents)
% Global Channels initialization
AgentNum=length(Agents);
GlobalChannels=Channels.empty();
for i=1:AgentNum
    for j=1:AgentNum
        GlobalChannels(i,j)=Channels(i,j,norm(Agents(i).pTrue_-Agents(j).pTrue_));
    end
end
end