#!/usr/bin/env python

import rospy
import actionlib
import time
from command_executer.msg import ExecuteCommandAction
from command_executer.msg import ExecuteCommandGoal, ExecuteCommandFeedback, ExecuteCommandResult
from command_executer.msg import CommandExecuterStatus, CommandState
from std_msgs.msg import String
from command_executer.shell_cmd import ShellCmd


def create_command(command, goal_idx,
                   action_server_goal=None,
                   start_time=None):
    """Create a dictionary representing the data of a command
    :param str command: The commandline command.
    :param ActionServerGoalHandle action_server_goal: The action server goal handle.
    :param float start_time: The timestamp of the starting time of the command.
    """
    as_g = action_server_goal
    cedict = {}
    if start_time is None:
        cedict['start_time'] = time.time()
    else:
        cedict['start_time'] = start_time
    cedict['finish_time'] = None
    cedict['command_duration'] = None
    cedict['command'] = command
    if as_g is not None:
        cedict['command_name'] = as_g.goal.goal.cmd_name
        cedict['command_description'] = as_g.goal.goal.cmd_description
        cedict['goal_handle'] = as_g
    else:
        # Generate a command name to be able to cancel it
        cedict['command_name'] = command.replace(
            ' ', '_') + "_from_topic_idx_" + str(goal_idx)
        cedict['command_description'] = "Command from the topic interface."
        cedict['goal_handle'] = None
    cedict['goal_idx'] = goal_idx
    cedict['shell_command'] = ShellCmd(command)
    return cedict


class CommandExecuter:
    def __init__(self):
        """Spawn a CommandExecuter server that offers an
        action server interface and a string topic interface to send commands
        to be executed. Also keeps track of them and publishes
        them on a topic to ease creation of GUIs from it.
        ROS parameters:
            check_commands_interval: float
                Check commands status every given seconds. (Default 0.1s)
            publish_status_interval: float
                Publish status of the commands every given seconds. (Default 1.0s)
            status_finished_commands_lifetime: float
                How long to keep publishing an old finished command in the
                status in seconds. (Default 120s)
            status_stdout_max_characters: int
                How many characters maximum you want published in the status
                topic for the stdout field.
            status_stderr_max_characters: int
                How many characters maximum you want published
                in the status topic for the stderr field."""
        self.check_commands_interval = rospy.get_param(
            "~check_commands_interval", 0.1)
        self.publish_status_interval = rospy.get_param(
            "~publish_status_interval", 1.0)
        self.status_finished_commands_lifetime = rospy.get_param(
            "~status_finished_commands_lifetime", 120.0)
        self.status_stdout_max_characters = rospy.get_param(
            "~status_stdout_max_characters", 200)
        self.status_stderr_max_characters = rospy.get_param(
            "~status_stderr_max_characters", 200)
        rospy.loginfo(
            "Initializing CommandExecuter with name: " + str(rospy.get_name()))
        rospy.loginfo("Checking commands on interval (s): " +
                      str(self.check_commands_interval))
        rospy.loginfo("Publishing states on interval (s): " +
                      str(self.publish_status_interval))
        rospy.loginfo("Finished commands lifetime (s): " +
                      str(self.status_finished_commands_lifetime))
        self.goal_idx = 0
        self.goals = []
        self.old_goals = []
        self._as = actionlib.ActionServer(rospy.get_name() + '/command',
                                          ExecuteCommandAction,
                                          goal_cb=self.execute_cb,
                                          auto_start=False)
        self._as.register_cancel_callback(self.cancel_cb)
        self._as.start()

        rospy.Timer(rospy.Duration(
            self.check_commands_interval), self.check_done)

        self._status_pub = rospy.Publisher('~commands_status',
                                           CommandExecuterStatus,
                                           queue_size=1)

        self._cancel_command = rospy.Subscriber('~cancel_command',
                                                String, self.topic_cancel_cb)

        self._execute_command = rospy.Subscriber('~execute_command',
                                                 String, self.topic_exec_cb)

        # Status should be diagnostics! in PAL that works pretty well!
        # Or at least do both
        rospy.Timer(rospy.Duration(self.publish_status_interval),
                    self.pub_status)

        rospy.Timer(rospy.Duration(self.publish_status_interval),
                    self.delete_old_finished_commands)

    # Smart stuff so we use the class with 'with'
    # so it cleans up when destroyed
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        rospy.loginfo("Stopping all active commands.")
        for g in self.goals:
            cmd = g.get('shell_command')
            cmd.kill()

    def execute_cb(self, goal):
        goal.set_accepted()
        goal_dict = create_command(command=goal.goal.goal.cmd,
                                   goal_idx=self.goal_idx,
                                   action_server_goal=goal)
        self.goal_idx += 1
        self.goals.append(goal_dict)

    def topic_exec_cb(self, data):
        goal_dict = create_command(command=data.data, goal_idx=self.goal_idx)
        self.goal_idx += 1
        self.goals.append(goal_dict)

    def topic_cancel_cb(self, data):
        cmd_name = data.data
        for g in self.goals[:]:
            if cmd_name == g.get('command_name'):
                cmd = g.get('shell_command')
                if cmd is None:
                    rospy.logwarn("We got a None shell command... fishy.")
                    continue
                cmd.kill()
                self.goals.remove(g)
                self.old_goals.append(g)
                g['finish_time'] = time.time()
                g['command_duration'] = g.get(
                    'finish_time') - g.get('start_time')

    def cancel_cb(self, goal):
        result = None
        # [:] creates a copy of the list
        for g in self.goals[:]:
            if goal.goal.goal.cmd_name == g.get('command_name'):
                cmd = g.get('shell_command')
                if cmd is None:
                    rospy.logwarn("We got a None shell command... fishy.")
                    continue
                cmd.kill()
                self.goals.remove(g)
                self.old_goals.append(g)
                g['finish_time'] = time.time()
                g['command_duration'] = g.get(
                    'finish_time') - g.get('start_time')
                result = ExecuteCommandResult()
                result.ret_val = cmd.get_retcode()
                if result.ret_val is None:
                    result.ret_val = -1  # Just in case something went wrong
                result.stdout = cmd.get_stdout()
                result.stderr = cmd.get_stderr()
        if result:
            goal.set_canceled(result)
        else:
            rospy.logerr("Errors canceling goal, cannot find it")

    def check_done(self, timer_event):
        # Iterate over a copy of the list, so we can modify self.goals
        for g in self.goals[:]:
            goal = g.get('goal_handle')
            cmd = g.get('shell_command')
            # Action server interface
            if goal is not None:
                feedback = ExecuteCommandFeedback()
                feedback.stdout = cmd.get_stdout()
                feedback.stderr = cmd.get_stderr()
                goal.publish_feedback(feedback)
                if cmd.is_done():
                    rospy.loginfo(
                        "Goal for {} is done. ".format(goal.goal.goal.cmd))
                    result = ExecuteCommandResult()
                    result.ret_val = cmd.get_retcode()
                    result.success = result.ret_val == 0
                    result.stdout = cmd.get_stdout()
                    result.stderr = cmd.get_stderr()
                    if result.ret_val == 0:
                        goal.set_succeeded(result)
                    else:
                        goal.set_aborted(result)
                    self.goals.remove(g)
                    self.old_goals.append(g)
                    g['finish_time'] = time.time()
                    g['command_duration'] = g.get(
                        'finish_time') - g.get('start_time')
            # Topic interface
            else:
                if cmd.is_done():
                    rospy.loginfo("Goal for " + g.get('command_name') +
                                  " is done. ")
                    self.goals.remove(g)
                    self.old_goals.append(g)
                    g['finish_time'] = time.time()
                    g['command_duration'] = g.get(
                        'finish_time') - g.get('start_time')

    def pub_status(self, timer_event):
        ces = CommandExecuterStatus()
        for g in self.goals:
            cmd = g.get('shell_command')
            cs = CommandState()
            cs.cmd = g.get('command')
            cs.cmd_name = g.get('command_name')
            cs.cmd_description = g.get('command_description')
            cs.ret_val = -1
            cs.stderr = cmd.get_stderr()[-self.status_stderr_max_characters:]
            cs.stdout = cmd.get_stdout()[-self.status_stdout_max_characters:]
            cs.time_running = rospy.Duration(time.time() - g.get('start_time'))
            ces.active_cmds.append(cs)

        for g in self.old_goals:
            cmd = g.get('shell_command')
            cs = CommandState()
            cs.cmd = g.get('command')
            cs.cmd_name = g.get('command_name')
            cs.cmd_description = g.get('command_description')
            cs.ret_val = cmd.get_retcode()
            if cs.ret_val is None:
                continue
            cs.stderr = cmd.get_stderr()[-self.status_stderr_max_characters:]
            cs.stdout = cmd.get_stdout()[-self.status_stdout_max_characters:]
            cs.time_running = rospy.Duration(g.get('command_duration'))
            ces.finished_cmds.append(cs)
        self._status_pub.publish(ces)

    def delete_old_finished_commands(self, timer_event):
        for g in self.old_goals[:]:
            if time.time() - g.get('finish_time') > self.status_finished_commands_lifetime:
                self.old_goals.remove(g)


if __name__ == '__main__':
    rospy.init_node('command_executer')
    # Using 'with' makes sure thate server is closed cleanly
    with CommandExecuter() as srv:
        rospy.spin()
