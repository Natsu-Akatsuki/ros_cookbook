#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import sys

if sys.version_info.major == 2:
    import Queue as queue
else:
    import queue
import rospy
import time
import curses
from collections import OrderedDict

from tf2_msgs.msg import TFMessage


class TfMonitor(object):
    '''
    Attributes
    ------------
    data : {FromTo: {PublisherName : TfStatistics}}g
    msgs : [(rospy.Time, TFMessage, bool)]
        list of tuple(received_stamp, message, is static)
    '''

    def __init__(self):
        # Attributes
        self.msgs = queue.Queue()
        self.data = OrderedDict()
        self.viewer = ViewerConsole()
        # Subscribers
        self.sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback, queue_size=1000)
        self.sub = rospy.Subscriber('/tf_static', TFMessage, self.tf_static_callback, queue_size=1000)

    def main(self):
        with self.viewer:
            while not rospy.is_shutdown():
                self.process_msgs()

    def process_msgs(self):
        '''Process /tf
        '''
        # Pop message from self.msgs and check stamps
        while self.msgs.qsize() > 0:
            recieved_stamp, tf_msg, static = self.msgs.get()
            # publisher = tf_msg._connection_header['callerid']
            publisher = PublisherName.from_msg(tf_msg)
            self.update_data(publisher, recieved_stamp, tf_msg, static=static)
        # Print results
        self.viewer.show(self.data)

    def update_data(self, publisher, stamp, tf_msg, static=False):
        '''Common process for both /tf and /tf_static

        Parameters
        -----------
        publisher : str
            Name of ros node who publish tf message
        stamp     : rospy.Time
            Timestamp when the tf message recived
        tf_msg    : TFMessage
            Message to process
        static    : bool
            True if static
        '''
        for transform_msg in tf_msg.transforms:
            from_to = FromTo.from_msg(transform_msg)
            if not from_to in self.data:
                self.data[from_to] = {}
            if not publisher in self.data[from_to]:
                self.data[from_to][publisher] = TransformStatictics(stamp, transform_msg, static=static)
            else:
                self.data[from_to][publisher].update(stamp, transform_msg, static=static)

    def tf_callback(self, msg):
        self.common_callback(msg, static=False)

    def tf_static_callback(self, msg):
        self.common_callback(msg, static=True)

    def common_callback(self, msg, static=False):
        received_stamp = rospy.Time.from_sec(time.time())
        self.msgs.put((received_stamp, msg, static))


class FromTo(object):
    '''
    Parameters
    -----------

    Examples:
    >>> FromTo('map', 'odom')
    '''

    def __init__(self, from_, to_):
        self.frm = from_
        self.to = to_

    @classmethod
    def from_msg(cls, msg):
        return cls(msg.header.frame_id, msg.child_frame_id)

    def __hash__(self):
        return hash(tuple([self.frm, self.to]))

    def __eq__(self, other):
        return hash(self) == hash(other)

    def __str__(self):
        return '{:20} -> {:20}'.format(self.frm, self.to)

    def __repr__(self):
        return '{:20} -> {:20}'.format(self.frm, self.to)


class PublisherName(str):
    @classmethod
    def from_msg(cls, msg):
        return cls(msg._connection_header['callerid'])


def get_standard_stamp():
    return rospy.Time.now()


class TransformStatictics(object):
    def __init__(self, recieved_stamp, msg, static=False):
        '''
        Parameters
        -----------
        recieved_stamp : rospy.Time
        msg : TransformStamped
        static : bool
            True if /tf_static. (default: False)

        Attributes
        -----------
        from : str
            Parent frame id of transformation
        to   : str
            Child frame id of transformation
        stamp_diff : float
            Difference between message timestamp and standard timestamp
        msg : PoseStamped
            Latest PoseStamped message
        '''
        self.static = static
        self.frm = msg.header.frame_id
        self.to = msg.child_frame_id
        self.prev_recieved_stamp = rospy.Time()
        self.curr_recieved_stamp = recieved_stamp
        self.hz = -1.
        self.stamp_diff_latest = (msg.header.stamp - get_standard_stamp()).to_sec()
        self.stamp_diff_max = self.stamp_diff_latest
        self.stamp_diff_min = self.stamp_diff_latest
        self.stamp_diff_ave = self.stamp_diff_latest
        self.n = 1
        self.msg = msg

    def update(self, recieved_stamp, msg, static=False):
        assert self.frm == msg.header.frame_id
        assert self.to == msg.child_frame_id
        assert self.static == static
        self.prev_recieved_stamp = self.curr_recieved_stamp
        self.curr_recieved_stamp = recieved_stamp
        try:
            self.hz = 1. / (self.curr_recieved_stamp - self.prev_recieved_stamp).to_sec()
        except ZeroDivisionError:
            self.hz = -1
        self.stamp_diff_latest = (msg.header.stamp - get_standard_stamp()).to_sec()
        self.n += 1
        self.stamp_diff_max = max(self.stamp_diff_latest, self.stamp_diff_max)
        self.stamp_diff_min = min(self.stamp_diff_latest, self.stamp_diff_min)
        self.stamp_diff_ave += (self.stamp_diff_latest - self.stamp_diff_ave) / self.n


class ViewerInterface(object):
    def __init__(self):
        pass

    def show(self, data):
        '''
        data : {FromTo: {PublisherName : TfStatistics}}
        '''
        pass

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, ex_type, ex_value, trace):
        if ex_type is not None:
            pass
        self.close()

    def open(self):
        pass

    def close(self):
        pass


class ViewerConsole(ViewerInterface):
    def __init__(self):
        self.top_index = 0

    def open(self):
        self.stdscr = curses.initscr()
        self.stdscr.timeout(100)
        curses.start_color()
        curses.use_default_colors()
        for i in range(1, curses.COLORS):
            curses.init_pair(i, i, -1)
        curses.noecho()
        # 需要运行于真实的终端
        curses.cbreak()
        self.colors = {
            'white': curses.COLOR_WHITE,
            'red': curses.COLOR_RED,
            'green': curses.COLOR_GREEN,
            'yellow': curses.COLOR_YELLOW,
            'blue': curses.COLOR_BLUE,
            'magenta': curses.COLOR_MAGENTA,
            'cyan': curses.COLOR_CYAN,
            'black': curses.COLOR_BLACK,
        }

    def close(self):
        curses.echo()
        curses.nocbreak()
        curses.endwin()

    def write_labels(self):
        self.stdscr.addstr(0, 0, '{:^37}|{:^26}|{:^6}|{:<20}'.format(
            'Frame ID', 'Stamp Difference [sec]', 'Freq', ' Node'), curses.A_UNDERLINE)
        self.stdscr.addstr(1, 0, '{:^18}|{:^18}|{:^8}|{:^8}|{:^8}|{:^6}|{:<20}'.format(
            'from', 'to', 'latest', 'min', 'max', '[Hz]', ''), curses.A_UNDERLINE)

    def show(self, data):
        '''
        Parameters
        -----------
        data : {FromTo: {PublisherName : TfStatistics}}
        '''
        size = self.get_data_size(data)
        data = OrderedDict(sorted(data.items(), key=lambda item: item[0].frm))
        max_row, max_col = self.stdscr.getmaxyx()
        self.write_labels()
        row = 1
        self.process_keyboard(size)
        self.stdscr.clear()
        parent_dict = self.generate_parent_dict(data)  # Check multiple parent
        for i, (ft, ft_data) in enumerate(data.items()):
            if i < self.top_index:
                continue
            color = self.colors['white']
            if len(data[ft]) > 1:
                color = self.colors['yellow']
            if len(parent_dict[ft.to]) > 1:
                color = self.colors['red']
            for pub, stat in ft_data.items():
                row += 1
                if row >= max_row - 1:
                    self.stdscr.refresh()
                    return
                if stat.static:
                    output = '{:<18}|{:<18}|{:=+.1e}|{:=+.1e}|{:=+.1e}|{:6.1f}|{}'.format(
                        ft.frm, ft.to,
                        stat.stamp_diff_latest, stat.stamp_diff_min, stat.stamp_diff_max,
                        1. / (stat.curr_recieved_stamp - stat.prev_recieved_stamp).to_sec(),
                        pub)
                else:
                    output = '{:<18}|{:<18}|{:=+8.3f}|{:=+8.3f}|{:=+8.3f}|{:6.1f}|{}'.format(
                        ft.frm, ft.to,
                        stat.stamp_diff_latest, stat.stamp_diff_min, stat.stamp_diff_max,
                        1. / (stat.curr_recieved_stamp - stat.prev_recieved_stamp).to_sec(),
                        pub)
                self.stdscr.addstr(row, 0, output, curses.color_pair(color))
        self.stdscr.addstr(max_row - 1, max(max_col - 30, 0), '[help] j:down, k:up q:quit')
        self.stdscr.refresh()

    def generate_parent_dict(self, data):
        '''
        Parameters
        -----------
            data : {FromTo: {PublisherName : TfStatistics}}
        Returns
        --------
            {str: [str]}
                key is frame name, value is list of parent frame names
        '''
        d = {}
        for ft in data:
            if ft.to not in d:
                d[ft.to] = [ft.frm]
            else:
                d[ft.to].append(ft.frm)
        return d

    def get_data_size(self, data):
        '''
        Parameters
        -----------
        data : {FromTo: {PublisherName : TfStatistics}}
        '''
        s = 0
        for d in data.values():
            s += len(d)
        return s

    def process_keyboard(self, size):
        key = self.stdscr.getch()
        if (key == ord('j')) and (self.top_index < size):
            self.top_index += 1
        elif (key == ord('k')) and (self.top_index > 0):
            self.top_index -= 1
        elif key == ord('q'):
            exit(0)


def main():
    checker = TfMonitor()
    checker.main()


if __name__ == '__main__':
    rospy.init_node("rocky_tf_monitor", log_level=rospy.INFO)
    main()
