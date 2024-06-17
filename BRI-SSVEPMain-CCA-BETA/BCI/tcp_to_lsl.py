from pylsl import StreamInfo, StreamOutlet


class send_marker(object):
    def __init__(self):
        info = StreamInfo('TCP_to_LSL', 'Markers', 1, 0, 'string', 'myuidw43536')
        self.outlet = StreamOutlet(info)

    def send(self, msg: str):
        self.outlet.push_sample([msg])
