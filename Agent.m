classdef Agent
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
        Deleyed_TX_;

        tx_start_time_;  % Transmission start time
        tx_delay_;      % Transmission delay

        tx_time_;       % Tx timestamp
        rx_time_;       % Rx timestamp

        tx_msg_;        % TX message
        rx_msg_;        % RX message
        
        slot_num_;      % Assigned slot number;
        
        master_;        % master anchor
        slave_;         % slave anchor (automatically rank by id)

       
        pos_buffer_;
        rxtime_buffer_;        % store message

        estRes_;        % Estimation results
    end
    
    methods
        function obj = Agent(id, pTure, vTrue)
            %AGENT Constructor
            obj.id_ = id;
            obj.pTrue_ = pTure;
            obj.vTrue_ = vTrue;
            obj.role_= 1;      % initialize as tag
            obj.slot_num_=0;   % initialize as taking up slot 0;

            obj.Deleyed_TX_=0; % default non-delayed transmit

            obj.skewTrue_=(rand-0.5)*40e-6;

            
            obj.pos_buffer_=zeros(10,2);
            obj.rxtime_buffer_=zeros(10,1);
           
        end

        function obj=uwbTx(obj,delayed_flag)
            if delayed_flag==0
                delayed_flag=1;
            else
                
            end

        end

        function obj=uwbRx(obj,channel)
            obj.rx_time_=channel.rx_;
            obj.rx_msg_=channel.uwb_msg_;
        end  
    end

end
