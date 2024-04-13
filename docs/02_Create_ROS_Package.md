[Back to Contents](../README.md)

## Create ROS package

You can do this either natively on your host or inside a container as you did on [Step 2](./01_ROS_Noetic_Containers.md#step-2-test-your-worskpace-image) in the previous section.

```bash
docker run --rm --name ros-master -it -v ${PWD}/catkin_ws/src:/home/ros/catkin_ws/src ros-noetic-workspace
```


### Step 1: Create a ROS Package

First, you need to create a ROS package. Open a terminal natively or in your container and navigate to your catkin workspace's `src` directory (create a new workspace if you don't have one):

```bash
cd ~/catkin_ws/src
```

Create a new package named `hello_world_pkg` with dependencies on `rospy` and `std_msgs`:

```bash
catkin_create_pkg hello_world_pkg rospy std_msgs
```

**Troubleshooting**:

Make sure ROS is installed an available

```
printenv | grep ROS
```

If no environment variables are set then try again running this first:

```
source /opt/ros/noetic/setup.bash
```

Also check if this is not set in `.bashrc` already:

```
cat ~/.bashrc | grep source /opt/ros/noetic/setup.bash
```

### Step 2: Write the Publisher Node

Navigate into your new package and create a script for the publisher:

```bash
cd hello_world_pkg
mkdir scripts
cd scripts
```

Create a file named `talker.py`:

```bash
touch talker.py
```

Edit `talker.py` using a text editor and add the following content:

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "Hello, world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

Make the script executable:

```bash
chmod +x talker.py
```

### Step 3: Write the Subscriber Node

In the same `scripts` directory, create a file named `listener.py`:

```bash
touch listener.py
```

Edit `listener.py` and add the following content:

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

Make the script executable:

```bash
chmod +x listener.py
```

### Step 4: Build the Package and Source the Workspace

Navigate back to the **root of your catkin workspace** and build the package:

```bash
cd ~/catkin_ws
catkin_make
```

Source the workspace to make sure the newly created package is found:

```bash
source devel/setup.bash
rospack list | grep hello 
```

Expected output:

```bash
hello_world_pkg /home/ros/catkin_ws/src/hello_world_pkg
```

**NOTE**: You can leave this container open for the next steps or you can close it for now.

---

[Previous: ROS Noetic with Docker Containers](./01_ROS_Noetic_Containers.md) | [Next: Running ROS Package Natively](./03_Running_ROS_Package_Natively.md)