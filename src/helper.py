import rosbag
import os


class RosbagRecorder(object):

    def __init__(self, filename_prefix, filename_suffix, default_topic_name='topic'):
        self.prefix = filename_prefix
        self.suffix = filename_suffix

        self.filename = ''
        self.recording = False
        self.default_topic_name = default_topic_name

    def write(self, msg, topic_name=None):
        """Write the message to the bag. """
        if self.recording:

            if topic_name is None:
                topic_name = self.default_topic_name

            try:
                self.bag.write(topic_name, msg)
            except ValueError as e:
                print('Error when writing to bag: {}'.format(e))

    def start(self):
        """Starts the recording. Creates a new bag. """
        if not self.recording:
            self.recording = True
            self.filename = get_unique_filename(self.prefix, self.suffix, padding=2)
            self.bag = rosbag.Bag(self.filename, 'w')

    def stop(self):
        """Stops the recording. Closes the bag. """
        if self.recording:
            self.recording = False
            self.bag.close()

    def get_filename(self):
        return self.filename


def get_unique_filename(prefix, suffix, padding=0):
    """Sets a filename on the form filename_prefixZ.bag where Z is the first free number.
    Pads with zeros, e.g. first free number 43 and padding=5 will give 00043. """
    __location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

    i = 0
    while os.path.exists(os.path.join(__location__, '{}{}{}'.format(
            prefix, str(i).zfill(padding), suffix))):
        i += 1

    filename = os.path.join(__location__, '{}{}{}'.format(
        prefix, str(i).zfill(padding), suffix))

    return filename


def print_numpy(a):
    s = '['
    for v in a.flatten():
        s += ' {:.2f}'.format(v)
    s += ' ]'

    print(s)