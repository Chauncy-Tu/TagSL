classdef Channels
    %CHANNELS UWB channels
    
    properties
        idTx_; % Transmitting id
        idRx_; % Receiving id
        tx_;   % Transmitting time
        rx_;   % Receiving time
        busy_;  % Either the channel is busy
        uwb_msg_;  % Message
        dGt_;  % True distance of the transmitting and receiving agents


        



%         dGt_;  % True distance of the transmitting and receiving agents
%         d_;    % Estimated distance
%         a_;    % Estimated pseudo-clock offset
%         b_;    % Estimated clock skew
%         aPropagated_; % Propagated clock offset until its next broadcast time
%         bPropagated_; % Propagated clock skew
%         aGt_;  % Ground truth pseudo-clock offset
%         bGt_;  % Ground truth pseudo-clock skew
%         sigmaRx_;
%         P_;    % Covariance matrix
%         slotInterval_;
%         slotCnt_;
    end
    
    methods
        function obj = Channels(idTx, idRx, dGt)
            %CHANNELS Constructor
            if (nargin > 0)
                obj.idTx_ = idTx;
                obj.idRx_ = idRx;
                obj.dGt_ = dGt;               
            else
                obj.idTx_ = -1;
                obj.idRx_ = -1;
                obj.dGt_ = 0;
            end
            obj.tx_=0;
            obj.rx_=0;
            obj.busy_=0;
        end          

%         function obj = Channels(idTx, idRx, tx, rx, aGt, bGt, dGt, ...
%                 sigmaRx, slotInterval, slotCnt)
%             %CHANNELS Constructor
%             if (nargin > 0)
%                 obj.idTx_ = idTx;
%                 obj.idRx_ = idRx;
%                 obj.tx_ = tx;
%                 obj.rx_ = rx;
%                 obj.dGt_ = dGt;
%                 obj.d_ = zeros(length(tx), 1);
%                 obj.aGt_ = aGt;
%                 obj.bGt_ = bGt;
%                 obj.a_ = zeros(length(tx), 1);  
%                 obj.aPropagated_ = zeros(length(tx), 1);
%                 obj.bPropagated_ = zeros(length(tx), 1);
%                 obj.b_ = zeros(length(tx), 1);  
%                 obj.sigmaRx_ = sigmaRx;
%                 obj.P_ = repmat([0.1 0; 0 1], [1,1, length(tx)]);
%                 obj.slotInterval_ = slotInterval;
%                 obj.slotCnt_ = slotCnt;
%             else
%                 obj.idTx_ = -1;
%                 obj.idRx_ = -1;
%             end
%         end
        
    end
end

