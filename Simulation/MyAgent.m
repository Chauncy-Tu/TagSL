classdef MyAgent1
    %AGENT Dynamic agent

    % including 3 types: anchor tag and listener
    

    properties
        % basic param
        id_;            % Agent ID
        role_;          % 0:Master Anchor  1:Slave Anchor  2:Tag

        % spatial param
        pTrue_;         % True xy position
        p_;             % Position with noise
        vTrue_;         % True xy velocity
   
        % temporal param
        offsetTrue_;    % True clock offset with respect to atom clock
        offset_;        % Clock offset with respect to atom clock
        skewTrue_;      % True clock skew with respect to atom clock
        
        % TX/RX param
        tx_start_time_;  % Transmission start time
        tx_delay_;      % Transmission delay

        tx_time_;       % Tx timestamp
        rx_time_;       % Rx timestamp

        tx_msg_;        % TX message
        rx_msg_;        % RX message

        slot_num_;      % Assigned slot number;
        

        channels_;      % Communicating channels between it and its neighbors

        estRes_;        % Estimation results
    end
    
    methods
        function obj = MyAgent(id, pTure, vTrue)
            %AGENT Constructor
            obj.id_ = id;
            obj.pTrue_ = pTure;
            obj.vTrue_ = vTrue;
            obj.role_= 1;      % initialize as tag
            obj.slot_num_=0;   % initialize as taking up slot 0;

            
%             obj.channels_ = Channels.empty();
%             for id = 1:slotCnt
%                 % TODO: add frameCnt to establish channels
%                 obj.channels_(id) = Channels();
%             end
        end

        function obj=uwbTx(obj,start_time,delay)
            obj.tx_start_time_= start_time;
            obj.tx_delay_=delay;
            obj.tx_time_=obj.tx_start_time_+obj.tx_delay_;

        end

        function obj=uwbRx(obj,channel)
            obj.rx_time_=channel.rx_;
            obj.rx_msg_=channel.uwb_msg_;
        end  
    end

end
