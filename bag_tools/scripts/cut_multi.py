#!/usr/bin/python
"""
Copyright (c) 2018,
FabolHak
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision Group, University of
      the Balearican Islands nor the names of its contributors may be used to
      endorse or promote products derived from this software without specific
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import yaml
import argparse


def cut_multi(filename):
    f = file(filename, 'r')
    for data in yaml.load_all(f):
        start = rospy.Time(data['start'])
        end = rospy.Time(data['end'])

        # get starting time of bag
        start_bag = rospy.Time.from_sec(999999999999)
        for topic, msg, t in rosbag.Bag(data['source'], 'r').read_messages():
            if t < start_bag:
                start_bag = t
        if start_bag > start:
            rospy.logwarn("   Provided time %s is smaller than bag start time %s. Use bag start time instead.",
                          start, start_bag)
            start = start_bag

        # get end time of bag
        end_bag = rospy.Time.from_sec(0)
        for topic, msg, t in rosbag.Bag(data['source'], 'r').read_messages():
            if t > end_bag:
                end_bag = t
        if end_bag < end:
            rospy.logwarn("   Provided time %s is bigger than bag en time %s. Use bag end time instead.",
                          end, end_bag)
            end = end_bag

        # write bag file
        outbag = rosbag.Bag(data['dest'], 'w')
        num_messages = 0
        rospy.loginfo('Extracting messages from: %s', data['source'])
        for topic, msg, t in rosbag.Bag(data['source'], 'r').read_messages(start_time=start, end_time=end):
            outbag.write(topic, msg, t)
            num_messages = num_messages + 1
        outbag.close()
        rospy.loginfo('New output bagfile has %s messages', num_messages)


if __name__ == "__main__":
  rospy.init_node('cut_multi')
  parser = argparse.ArgumentParser(
      formatter_class=argparse.RawDescriptionHelpFormatter,
      description='Cuts out a section from multiple input bagfiles and writes it to output bagfiles.\n'
                  'Needs a .yaml file with the cutting details. The yaml file should look like:\n\n'
                  '  source: <source_bag_1>\n'
                  '  dest:   <output_bag_1>\n'
                  '  start:  <unix_start_time_1>\n'
                  '  end:    <unix_end_time_1>\n'
                  '  ---\n'
                  '  source: <source_bag_2>\n'
                  '  dest:   <output_bag_2>\n'
                  '  start:  <unix_start_time_2>\n'
                  '  end:    <unix_end_time_2>\n')
  parser.add_argument('--file', help='.yaml file with all cutting information', required=True)
  args = parser.parse_args()
  try:
    cut_multi(args.file)
  except Exception, e:
    import traceback
    traceback.print_exc()
