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

from pathlib import Path
import re

class SmartCage():
    def __init__(self, logger):
        self.logger = logger
        self.training_period_prefix = 'training_period_'

    def start_new_training_period(self, mouse_name):
        self.base_path = Path.home() / 'smart_cage_data'
        self.base_path = self.base_path / mouse_name
        try:
            self.base_path.mkdir(parents=True)
        except FileExistsError:
            pass
        training_period_names = sorted([x.name for x in self.base_path.iterdir() if x.is_dir()])
        try:
            last_training_period_name = training_period_names[-1]
            last_training_period_number = int(re.findall(self.training_period_prefix + '(\d+)', last_training_period_name)[0])
            new_training_period_number = last_training_period_number + 1
        except IndexError:
            new_training_period_number = 0
        new_training_period_name = f'{self.training_period_prefix}{new_training_period_number:03}'
        self.base_path = self.base_path / new_training_period_name
        self.base_path.mkdir()
        self.logger.info(f'New training period base path: {self.base_path}')
