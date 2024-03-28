#!/usr/bin/env python3

import subprocess
import sys

def build_image(dockerfile_path, tag):
    """
    Build a Docker image from a specified Dockerfile.

    :param dockerfile_path: The path to the Dockerfile.
    :param tag: The tag to apply to the built image.
    """
    try:
        subprocess.check_call(["docker", "build", "-f", dockerfile_path, "-t", tag, "."])
        print(f"Successfully built {tag}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to build {tag}: {e}", file=sys.stderr)
        sys.exit(1)

def main():
    images = [
        ("Dockerfile", "ros-workspace:latest"),
    ]

    for dockerfile, tag in images:
        build_image(dockerfile, tag)

if __name__ == "__main__":
    main()
