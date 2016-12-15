#!/usr/bin/env python

from __future__ import print_function
import rospy
from genpy.message import fill_message_args
import time
import yaml
from actionlib import SimpleActionClient
from command_executer.msg import ExecuteCommandAction, ExecuteCommandGoal


def yaml_to_msgs(filename):
    """
    Given a filename of a YAML file with a list
    of ExecuteCommandGoals return the messages filled.
    """
    f = open(filename, 'r')
    yaml_doc = yaml.load(f.read())
    msgs = []
    for msg_dict in yaml_doc:
        if not type(msg_dict) == dict:
            raise ValueError("yaml file " + filename +
                             " does not contain a list of dictionaries")
        m = ExecuteCommandGoal()
        fill_message_args(m, msg_dict)
        msgs.append(m)
    return msgs


if __name__ == '__main__':
    argv = rospy.myargv()
    if len(argv) != 3:
        print("Usage:")
        print(argv[0] +
              " <command executer action server> <yaml file with list of commands>")
        print("")
        print("Example:")
        print(argv[0] + " /main_computer/command list_of_commands.yaml")
        exit(0)

    print("Loading commands from " + argv[2])
    msgs = yaml_to_msgs(argv[2])
    print("Loaded " + str(len(msgs)) + " commands.")

    print("Connecting to action server: " + argv[1])
    rospy.init_node('commands_from_yaml')
    ac = SimpleActionClient(argv[1], ExecuteCommandAction)
    d = rospy.Duration(5.0)
    while not rospy.is_shutdown() and not ac.wait_for_server(d):
        print("Waiting for action server " + argv[1] + " ...")

    for msg in msgs:
        print("\nSending command:\n" + str(msg)[:200] + "...")
        ac.send_goal(msg)
        time.sleep(0.1)

    print("All commands sent.")
