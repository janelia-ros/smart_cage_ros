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

from pathlib import Path

import cv2
import time
import math
import datetime

class ImageDataWriterNode(DataWriterNode):
    def __init__(self):
        super().__init__('image_data_writer')

        self.cap = None
        self._image_timer = None
        self._image_timer_period = 1

    def _data_writer_control_callback(self, msg):
        super()._data_writer_control_callback(msg)
        self.base_path = self.base_path / 'images'
        try:
            self.base_path.mkdir(parents=True)
        except FileExistsError:
            pass
        self.logger.info('saving images into: ' + str(self.base_path))
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3,320)
        self.cap.set(4,240)
        self._image_timer = self.create_timer(self._image_timer_period, self._image_timer_callback)

    def _image_timer_callback(self):
        ret, frame = self.cap.read()
        now = time.time()
        now_frac, now_whole = math.modf(time.time())
        nanosec = int(now_frac * 1e9)
        file_name = datetime.datetime.fromtimestamp(now).strftime('%H-%M-%S')
        file_name = file_name + '-{0}'.format(nanosec)
        file_path = str(self.base_path / '{0}.jpg'.format(file_name))
        self.logger.info(file_path)
        cv2.imwrite(file_path, frame)

    def close(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    image_data_writer_node = ImageDataWriterNode()

    try:
        rclpy.spin(image_data_writer_node)
    except KeyboardInterrupt:
        pass

    image_data_writer_node.close()
    image_data_writer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
