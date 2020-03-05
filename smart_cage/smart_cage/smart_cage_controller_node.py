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

from smart_cage_msgs.msg import TrainingPeriod, Session

from .smart_cage_controller import SmartCageController

class SmartCageControllerNode(Node):
    def __init__(self):
        super().__init__('smart_cage_controller')
        self.logger = self.get_logger()
        self._smart_cage_controller = SmartCageController(self.logger)

        self._create_training_period_subscription = self.create_subscription(
            TrainingPeriod,
            'create_training_period',
            self._create_training_period_callback)
        self._create_training_period_subscription  # prevent unused variable warning

        self._create_session_subscription = self.create_subscription(
            Session,
            'create_session',
            self._create_session_callback)
        self._create_session_subscription  # prevent unused variable warning

    def _create_training_period_callback(self, msg):
        self._smart_cage_controller.create_training_period(msg)

    def _create_session_callback(self, msg):
        self._smart_cage_controller.create_session(msg)

def main(args=None):
    rclpy.init(args=args)

    smart_cage_controller_node = SmartCageControllerNode()

    try:
        rclpy.spin(smart_cage_controller_node)
    except KeyboardInterrupt:
        pass

    smart_cage_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
