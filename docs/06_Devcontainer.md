[Back to Contents](../README.md)

# Use the devcontainer for Debugging


### Prerequisites
- Ensure you have **VS Code**, **Docker**, and the **Remote - Containers extension** installed and set up on your machine.

### Steps to Open/Start a Devcontainer

1. **Open VS Code**: Launch Visual Studio Code on your computer.

2. **Open Your Project**: 
   - Use `File > Open Folder...` to open the folder containing your project.
   - Make sure your project folder includes a `.devcontainer` directory with a `devcontainer.json` file. This file defines your container configuration.

3. **Start the Devcontainer**: 
   - Open the **Command Palette** (`Ctrl+Shift+P` or `Cmd+Shift+P` on macOS).
   - Type and select `Remote-Containers: Reopen in Container`. This command instructs VS Code to build (if necessary) and start the container defined by your `devcontainer.json`.

4. **Wait for Container to Initialize**: VS Code will automatically build the Docker image (if it's not already built) and start the container based on the specifications in your `devcontainer.json`. This process may take a few minutes.

5. **Develop Inside the Container**: Once the container is running, you can start editing your code, running your applications, and using VS Code as if everything were installed locally on your machine.

### Troubleshooting Tips

- **Container Doesn't Start**: Check your Docker Desktop or Docker Engine is running. Also, review your `devcontainer.json` for any syntax errors.

- **Extensions Not Working**: Ensure required extensions are listed in the `extensions` section of your `devcontainer.json`.

- **Performance Issues**: Docker performance can vary based on your file system and Docker settings. Consider Docker's documentation for performance tuning specific to your OS.



---

[Previous: Running ROS package with a Multiple Containers](./05_Runnning_ROS_Package_Multiple_Containers.md) | [Next: Using ROS with graphical apps](./07_ROS_GUI_Apps.md)