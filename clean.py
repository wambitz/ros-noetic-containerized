import subprocess

def run_command(command):
    """
    Executes a shell command and captures any errors.

    :param command: The command to run as a list.
    """
    try:
        subprocess.check_call(command)
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {' '.join(command)}", e)

def remove_container(container_name):
    """
    Stops and removes a Docker container.

    :param container_name: The name of the container to remove.
    """
    print(f"Stopping and removing container: {container_name}")
    run_command(["docker", "container", "stop", container_name])
    run_command(["docker", "container", "rm", container_name])

def remove_network(network_name):
    """
    Removes a Docker network.

    :param network_name: The name of the network to remove.
    """
    print(f"Removing network: {network_name}")
    run_command(["docker", "network", "rm", network_name])

def main():
    container_names = ["ros-master", "ros-talker", "ros-listener"]
    network_name = "ros-network"

    for container_name in container_names:
        remove_container(container_name)

    remove_network(network_name)

if __name__ == "__main__":
    main()
