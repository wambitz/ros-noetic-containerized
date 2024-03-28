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


def is_container_exist(container_name):
    """
    Checks if a Docker container exists.

    :param container_name: The name of the Docker container.
    :return: True if the container exists, False otherwise.
    """
    try:
        subprocess.check_output(["docker", "inspect", "--type=container", container_name])
        return True
    except subprocess.CalledProcessError:
        return False


def remove_container(container_name):
    """
    Stops and removes a Docker container if it exists.

    :param container_name: The name of the container to remove.
    """
    if not is_container_exist(container_name):
        print(f"Container '{container_name}' does not exist.")
        return

    print(f"Stopping and removing container: {container_name}")
    run_command(["docker", "container", "stop", container_name])
    run_command(["docker", "container", "rm", container_name])


def is_network_exist(network_name):
    """
    Checks if a Docker network exists.

    :param network_name: The name of the Docker network.
    :return: True if the network exists, False otherwise.
    """
    try:
        subprocess.check_output(["docker", "network", "inspect", network_name])
        return True
    except subprocess.CalledProcessError:
        return False


def remove_network(network_name):
    """
    Removes a Docker network if it exists.

    :param network_name: The name of the network to remove.
    """
    if not is_network_exist(network_name):
        print(f"Network '{network_name}' does not exist.")
        return

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
