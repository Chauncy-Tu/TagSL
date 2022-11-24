classdef uwb_msg
    properties
        frame_type_;
        frame_cnt_;
        slot_cnt_;
        tx_id_;     % id of msg transmitter
        mt_id_;     % master id:for slave_anchor
        t_reply_;
        pos_;
        slave_;
        user_data_;
        crc_;
    end

    methods
        function obj = uwb_msg(frame_type,frame_cnt,slot_cnt,tx_id)
            obj.frame_type_=frame_type;
            obj.frame_cnt_=frame_cnt;
            obj.slot_cnt_=slot_cnt;
            obj.tx_id_=tx_id;
            obj.mt_id_=0;
            obj.t_reply_=0;
            obj.pos_=[0,0];
            obj.user_data_=zeros(10,1);
            obj.crc_=zeros(2,1);
        end
    end
end