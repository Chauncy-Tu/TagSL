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
        local_t_;       % local clock time
        
        % TX/RX param
        Deleyed_TX_;

        tx_start_time_;  % Transmission start time
        tx_delay_;      % Transmission delay

        tx_time_a_;       % Tx timestamp: atom time
        rx_time_a_;       % Rx timestamp: atom time
        tx_time_l_;       % Tx timestamp: local time
        rx_time_l_;       % Rx timestamp: local time

        tx_msg_;        % TX message
        rx_msg_;        % RX message

        cfo_;           % relative carrier frequency offset
        t_reply_;
        treply_count;

        
        slot_num_;      % Assigned slot number;
        
        master_;        % master anchor
        slave_;         % slave anchor (automatically rank by id)

       
        pos_buffer_;
        pos_count_;
        rxtime_buffer_;        % store message
        rxtime_count_;

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

            obj.offsetTrue_=(rand-0.5)*40e-6;  % ea
            obj.skewTrue_=(rand-0.5)*2e-7;   % 1e-7 s:30m clock skew
            
            obj.treply_count=0;
            obj.pos_count_=0;
            obj.rxtime_count_=0;
            
            
           
        end

        function obj=uwbTx(obj,delayed_flag)
            if delayed_flag==0
                
            else
                
            end

        end

        function obj=uwbRx(obj,channel)
            obj.rx_time_l_=channel.rx_*(1+obj.offsetTrue_)+obj.skewTrue_;
            obj.rx_time_a_=channel.rx_;
            obj.rx_msg_=channel.uwb_msg_;
        end  
    end

end
