# command_executer

Server to execute commands on the machine it is running.

The server keeps tracks of the commands so you can check the stdout, stderr and kill them.

It also publishes feedback on what commands are being executed and have been executed.

Offers an action server and a string topic interface to execute commands.

Useful for automating launching nodes/services/scripts from code in different machines
or to boot up a robot with a set of software.

# Usage

Make a launch file and give the node a well named namespace
```
<launch>

</launch>
```

When launched you can either send a command to be executed and track it with action server
interface or with the topic interface if you don't need to track it or you'll track it
from the status publishing interface.


