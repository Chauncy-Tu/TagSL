classdef uwb_msg
    properties
        frame_type_;
        frame_cnt_;
        slot_cnt_;
        t_reply_;
        pos_;
        slave_;
        user_data_;
        crc_;
    end

    methods
        function obj = uwb_msg(frame_type,frame_cnt,slot_cnt)
            obj.frame_type_=frame_type;
            obj.frame_cnt_=frame_cnt;
            obj.slot_cnt_=slot_cnt;
            obj.t_reply_=0;
            obj.pos_=[0,0];
            obj.user_data_=zeros(10,1);
            obj.crc_=zeros(2,1);
        end
    end
end