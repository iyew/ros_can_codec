#!/usr/bin/env python3

import os
import can
import rospy
import asyncio
import cantools
import importlib
from cantools.database.can.signal import NamedSignalValue


class Codec:
    def __init__(self):
        print('initialize cancodec..\n')
        SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
        DBC_FILE_PATH = os.path.join(SCRIPT_DIR, '..', 'dbc', 'D2S_CAN.dbc')
        self.can_db = cantools.database.load_file(DBC_FILE_PATH)
        self.bus0 = can.interface.Bus('can0', bustype='socketcan')
        #self.bus1 = can.interface.Bus('can1', bustype='socketcan')
        self.cancodec_msgs = importlib.import_module('ros_cancodec.msg')
        msg_names = [msg.name for msg in self.can_db.messages]
        msg_list = [getattr(self.cancodec_msgs, name) for name in msg_names]

        rospy.init_node('codec', anonymous=True)
        self.publishers = {name: rospy.Publisher(name+'_pub', msg, queue_size=10) for name, msg in zip(msg_names, msg_list)}
        self.subscribers = {name: rospy.Subscriber(name+'_sub', msg, self.send) for name, msg in zip(msg_names, msg_list)}


    def send(self, msg):
        msg_type = msg._type.split('/')[1]
        print('subscribed message: ', msg)
        detected_message = self.can_db.get_message_by_name(msg_type)
        sig_values = [getattr(msg, sig) for sig in msg.__slots__]
        decoded_signal = {name: val for name, val in zip(msg.__slots__, sig_values)}
        encoded_frame = detected_message.encode(decoded_signal)
        raw_frame = can.Message(arbitration_id=detected_message.frame_id, data=encoded_frame, is_extended_id=False)
        print('encoded raw_frame: ', raw_frame)
        self.bus0.send(raw_frame)
        print('send raw_frame to can0 success\n----------------------')


    async def receive(self):
        reader = can.AsyncBufferedReader()
        listeners = [
            self.decode,  # Callback function
            reader        # AsyncBufferedReader() listener
            #logger        # Regular listener object
        ]

        loop = asyncio.get_event_loop()
        notifier = can.Notifier(self.bus0, listeners, loop=loop)

        while not rospy.is_shutdown():
            try:
                raw_frame = await reader.get_message()
            except rospy.ROSInterruptException:
                #reader.on_message_received(raw_frame)
                notifier.stop()
                loop.close()


    def decode(self, raw_frame):
        print('received can frame from can0: ', raw_frame)
        detected_message = self.can_db.get_message_by_frame_id(int(raw_frame.arbitration_id))
        raw_signal = detected_message.decode(bytes(raw_frame.data)) 
        decoded_signal = {key: str(val) if isinstance(val, NamedSignalValue) else val for key, val in raw_signal.items()}
        DetectedSignal = getattr(self.cancodec_msgs, detected_message.name)
        signal = DetectedSignal(**decoded_signal)
        print('decoded signal: ', signal)
        self.publishers[detected_message.name].publish(signal)
        print('publish success\n----------------------')





if __name__ == '__main__':
    codec = Codec()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(codec.receive())
    loop.close()
