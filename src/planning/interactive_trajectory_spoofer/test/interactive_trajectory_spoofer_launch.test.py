# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

from launch import LaunchDescription

from launch_ros.actions import Node
import launch_testing

import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description():

    interactive_trajectory_spoofer = Node(
        package="interactive_trajectory_spoofer",
        executable="interactive_trajectory_spoofer_node_exe",
        name="test",
        parameters=[
            {
            }
        ],
    )

    context = {'interactive_trajectory_spoofer': interactive_trajectory_spoofer}

    return LaunchDescription([
        interactive_trajectory_spoofer,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, interactive_trajectory_spoofer):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(
            proc_info, [-15], process=interactive_trajectory_spoofer)