# inspired by http://answers.ros.org/question/49069/is-there-a-tfmessagefilter-in-the-tf-python-api/
import rospy

import Queue
import tf2_ros
import rospy
from message_filters import SimpleFilter

class TfMessageFilter(SimpleFilter):
    """Stores a message unless corresponding transforms is 
    available
    """
    def __init__(self, input_filter, base_frame, target_frame,
                 queue_size=500, tf_buf_sz=10.0):
        SimpleFilter.__init__(self)
        self.connectInput(input_filter)
        self.base_frame = base_frame
        self.target_frame = target_frame
        # TODO: Use a better data structure
        self.message_queue = Queue.Queue(maxsize=queue_size)
        self.tf2_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(tf_buf_sz))
        self.listener = tf2_ros.TransformListener(self.tf2_buffer)
        self.max_queue_size = queue_size
        self._max_queue_size_so_far = 0
        self.last_sent_msg_stamp = None

    def connectInput(self, input_filter):
        self.incoming_connection = \
                input_filter.registerCallback(self.input_callback)

    def poll_transforms(self, latest_msg_tstamp):
        """
        Poll transforms corresponding to all messages. If found throw older
        messages than the timestamp of transform just found
        and if not found keep all the messages.
        """
        # Check all the messages for transform availability
        tmp_queue = Queue.Queue(self.max_queue_size)
        first_iter = True
        # Loop from old to new
        while not self.message_queue.empty():
            msg = self.message_queue.get()
            tstamp = msg.header.stamp
            if self.last_sent_msg_stamp and tstamp < self.last_sent_msg_stamp:
                rospy.logwarn("Laser scan msg was dropped")
                continue
            if (first_iter and 
                self.message_queue.qsize() > self._max_queue_size_so_far):
                first_iter = False
                self._max_queue_size_so_far = self.message_queue.qsize()
                rospy.logdebug("Queue(%d) time range: %f - %f" %
                              (self.message_queue.qsize(), 
                               tstamp.secs, latest_msg_tstamp.secs))
                rospy.loginfo("Maximum queue size used:%d" %
                              self._max_queue_size_so_far)
            if self.tf2_buffer.can_transform(self.base_frame, self.target_frame,
                                          tstamp):
                transform = self.tf2_buffer.lookup_transform(self.base_frame, self.target_frame, tstamp).transform
                self.signalMessage(msg, (transform.translation, transform.rotation))
                # Note that we are deliberately throwing away the messages
                # older than transform we just received
                self.last_sent_msg_stamp = msg.header.stamp
                return
            else:
                # if we don't find any transform we will have to recycle all
                # the messages
                tmp_queue.put(msg)
        self.message_queue = tmp_queue

    def input_callback(self, msg):
        """ Handles incoming message """
        if self.message_queue.full():
            # throw away the oldest message
            rospy.logwarn("Queue too small. If you this message too often"
                          + " consider increasing queue_size")
            self.message_queue.get()

        self.message_queue.put(msg)
        # This can be part of another timer thread
        # TODO: call this only when a new/changed transform
        self.poll_transforms(msg.header.stamp)
