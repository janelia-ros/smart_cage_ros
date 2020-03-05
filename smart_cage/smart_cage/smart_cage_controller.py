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
import datetime
import json

class SmartCageController():
    def __init__(self, logger):
        self.logger = logger
        self.base_path = Path.home() / 'smart_cage_data'

        self.training_period_path = None
        self.training_period_prefix = 'training_period_'
        self.training_period_info_filename = 'training_period_info.json'
        self.training_period_state_filename = 'training_period_state.json'

        self.session_prefix = 'session_'
        self.session_info_filename = 'session_info.json'

    def get_mouse_path(self, mouse_name):
        return self.base_path / mouse_name

    def get_last_training_period_name(self, mouse_path):
        training_period_names = sorted([x.name for x in mouse_path.iterdir() if x.is_dir()])
        try:
            last_training_period_name = training_period_names[-1]
        except IndexError:
            last_training_period_name = None
        return last_training_period_name

    def get_last_training_period_path(self, mouse_path):
        last_training_period_name = self.get_last_training_period_name(mouse_path)
        if last_training_period_name is not None:
            last_training_period_path = mouse_path / last_training_period_name
        else:
            last_training_period_path = None
        return last_training_period_path

    def create_training_period(self, mouse_name, latch_durations):
        mouse_path = self.get_mouse_path(mouse_name)
        try:
            mouse_path.mkdir(parents=True)
        except FileExistsError:
            pass

        last_training_period_name = self.get_last_training_period_name(mouse_path)
        if last_training_period_name is not None:
            last_training_period_number = int(re.findall(self.training_period_prefix + '(\d+)', last_training_period_name)[0])
            new_training_period_number = last_training_period_number + 1
        else:
            new_training_period_number = 0
        new_training_period_name = f'{self.training_period_prefix}{new_training_period_number:03}'
        self.training_period_path = mouse_path / new_training_period_name
        self.training_period_path.mkdir()
        self.logger.info(f'New training period path: {self.training_period_path}')

        training_period_info = {}
        training_period_creation_datetime = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        training_period_info['training_period_creation_datetime'] = training_period_creation_datetime
        training_period_info['latch_durations'] = latch_durations
        self.training_period_info_path = self.training_period_path / self.training_period_info_filename
        with open(str(self.training_period_info_path), 'w') as f:
            json.dump(training_period_info, f, indent=2, sort_keys=True)
            self.logger.info(f'Wrote training period info to: {self.training_period_info_path}')

        training_period_state = {}
        training_period_state['latch_durations'] = latch_durations
        self.training_period_state_path = self.training_period_path / self.training_period_state_filename
        with open(str(self.training_period_state_path), 'w') as f:
            json.dump(training_period_state, f, indent=2, sort_keys=True)
            self.logger.info(f'Wrote training period state to: {self.training_period_state_path}')

    def get_last_session_name(self, last_training_period_path):
        session_names = sorted([x.name for x in last_training_period_path.iterdir() if x.is_dir()])
        try:
            last_session_name = session_names[-1]
        except IndexError:
            last_session_name = None
        return last_session_name

    def get_last_session_path(self, last_training_period_path):
        last_session_name = self.get_last_session_name(last_training_period_path)
        if last_session_name is not None:
            last_session_path = last_training_period_path / last_session_name
        else:
            last_session_path = None
        return last_session_path

    def create_session(self, mouse_name):
        try:
            mouse_path = self.get_mouse_path(mouse_name)
            last_training_period_path = self.get_last_training_period_path(mouse_path)
            if last_training_period_path is None:
                raise FileNotFoundError
        except FileNotFoundError:
            self.logger.error('Must create new training period before creating new session!')
            return

        last_session_name = self.get_last_session_name(last_training_period_path)
        if last_session_name is not None:
            last_session_number = int(re.findall(self.session_prefix + '(\d+)', last_session_name)[0])
            new_session_number = last_session_number + 1
        else:
            new_session_number = 0
        new_session_name = f'{self.session_prefix}{new_session_number:03}'
        last_training_period_path = self.get_last_training_period_path(mouse_path)
        self.session_path = last_training_period_path / new_session_name
        self.session_path.mkdir()
        self.logger.info(f'New session path: {self.session_path}')

        session_info = {}
        session_creation_datetime = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        session_info['session_creation_datetime'] = session_creation_datetime
        self.session_info_path = self.session_path / self.session_info_filename
        with open(str(self.session_info_path), 'w') as f:
            json.dump(session_info, f, indent=2, sort_keys=True)
            self.logger.info(f'Wrote session info to: {self.session_info_path}')
