# Copyright (c) 2020, Howard Hughes Medical Institute
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node

from .data_writer_node import DataWriterNode

from smart_cage_msgs.msg import LickportState

from pathlib import Path
import datetime
import csv

class LickportDataWriterNode(DataWriterNode):
    def __init__(self):
        super().__init__('lickport_data_writer')

        self.data_path = self.base_path / 'lickport_state.txt'
        self.logger.info('saving data into: ' + str(self.data_path))
        if self.data_path.exists():
            self.data_path_created = False
        else:
            self.data_path_created = True
        self.data_file = open(self.data_path, 'a', newline='')
        self.data_writer = None

        self._lickport_state_subscription = self.create_subscription(
            LickportState,
            'lickport_state',
            self._lickport_state_callback)
        self._lickport_state_subscription  # prevent unused variable warning

    def _lickport_state_callback(self, msg):
        if self.data_writer is None:
            self.fieldnames = [field[1:] for field in msg.__slots__]
            self.data_writer = csv.DictWriter(self.data_file,
                                              delimiter=' ',
                                              quotechar='|',
                                              quoting=csv.QUOTE_MINIMAL,
                                              fieldnames=self.fieldnames)
            if self.data_path_created:
                self.data_writer.writeheader()

        if self.save_data:
            msg_dict = {field[1:]: getattr(msg, field[1:]) for field in msg.__slots__}
            self.data_writer.writerow(msg_dict)

    def close_files(self):
        self.data_file.close()

def main(args=None):
    rclpy.init(args=args)

    lickport_data_writer_node = LickportDataWriterNode()

    rclpy.spin(lickport_data_writer_node)

    lickport_data_writer_node.close_files()
    lickport_data_writer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
