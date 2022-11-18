function Agents = Agents_init(AgentNum,M_Anchor_Index,S_Anchor_Index,Tag_Index,varargin)
% Agents initialization


% position and velocity initialization

if nargin==4
% AgentPos0=[[0 0];[5 0];[0 5];[-5 0];[0 -5];[3 3];[0 3];[4 2];[3 1];[-3 5]]*10;
    AgentPos0=[[0 0];[5 0];[0 5];[-5 0];[0 -5];[3 3]]*10;
    AgentV0=zeros(AgentNum,2);  % static mode
    Agents=Agent.empty();
    for i=1:AgentNum
        Agents(i)=Agent(i,AgentPos0(i,:),AgentV0(i,:));  
    end
else
    AgentPos0=varargin{1};
    AgentV0=varargin{2};
    Agents=Agent.empty();
    for i=1:AgentNum
        Agents(i)=Agent(i,AgentPos0(i,:),AgentV0(i,:));  
    end
end

% role initialization

for i=1:length(M_Anchor_Index)
    Agents(M_Anchor_Index(i)).role_=0;
end
for i=1:length(S_Anchor_Index)
    Agents(S_Anchor_Index(i)).role_=1;
end
for i=1:length(Tag_Index)
    Agents(Tag_Index(i)).role_=2;
end

% slot initialization (only for M_Anchor)
for i=1:length(M_Anchor_Index)
    Agents(M_Anchor_Index(i)).slot_num_=i;

    % determine the slave anchor for each master anchor and their number
    Agents(M_Anchor_Index(i)).slave_=sort(S_Anchor_Index);
  
end

% allocate master for slave_anchor:other algorithm
for i=1:length(S_Anchor_Index)
    Agents(S_Anchor_Index(i)).master_=sort(M_Anchor_Index);
end

end