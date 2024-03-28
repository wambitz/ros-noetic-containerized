import os
import subprocess

def run_command(command):
    """
    Runs a shell command and handles errors.

    :param command: The command to run as a list.
    """
    try:
        subprocess.check_call(command)
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {' '.join(command)}", e)

def create_docker_network(network_name):
    """
    Creates a Docker network if it does not already exist.

    :param network_name: The name of the Docker network.
    """
    existing_networks = subprocess.check_output(["docker", "network", "ls", "--format", "{{.Name}}"]).decode().splitlines()
    if network_name not in existing_networks:
        run_command(["docker", "network", "create", "--driver", "bridge", network_name])
    else:
        print(f"Network '{network_name}' already exists.")


def is_container_running(name):
    """
    Checks if a container is already running.

    :param name: The name of the Docker container.
    :return: True if the container is running, False otherwise.
    """
    try:
        output = subprocess.check_output(["docker", "ps", "--filter", f"name={name}", "--format", "{{.Names}}"]).decode().strip()
        return name in output
    except subprocess.CalledProcessError:
        return False


def run_container(name, network=None, command="/bin/bash", volumes=None, environment=None):

    if is_container_running(name):
        print(f"Container '{name}' is already running.")
        return
        
    try:
        cmd = ["docker", "run", "-d", "--rm", "--name", name, "--network", network]
        if volumes:
            for host_path, container_path in volumes.items():
                cmd.extend(["-v", f"{os.getcwd()}:{container_path}"])
        if environment:
            for key, value in environment.items():
                cmd.extend(["-e", f"{key}={value}"])
        cmd.append("ros-workspace")
        cmd.append(command)
        
        subprocess.run(cmd, check=True)
        print(f"Container '{name}' is running.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to run container '{name}'. Error: {e}")


if __name__ == "__main__":
    network_name = "ros-network"
    pwd = os.getenv("PWD")

    create_docker_network(network_name)

    # Start containers in separate threads
    # threads = []
    # for image_name, container_name in containers_to_start:
    #     thread = threading.Thread(target=start_container, args=(image_name, container_name))
    #     thread.start()
    #     threads.append(thread)

    # # Wait for all threads to complete
    # for thread in threads:
    #     thread.join()

    # print("All containers started.")


    # NOTE: This can be debug by docker logs <container_name> for now    
    run_container("ros-master", network_name, "roscore")
    
    run_container(
        "ros-listener",
        network_name,
        "catkin_make && source devel/setup.bash && rosrun hello_world_pkg listener.py",
        volumes={f"{pwd}/catkin_ws/src": "/home/ros/catkin_ws/src"},
        environment={"ROS_MASTER_URI": "http://ros-master:11311"}
    )
    
    run_container(
        "ros-talker",
        network_name,
        "catkin_make && source devel/setup.bash && rosrun hello_world_pkg talker.py",
        volumes={f"{pwd}/catkin_ws/src": "/home/ros/catkin_ws/src"},
        environment={"ROS_MASTER_URI": "http://ros-master:11311"}
    )
