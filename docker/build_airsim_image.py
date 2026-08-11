import argparse
import os
import subprocess


def main():
    parser = argparse.ArgumentParser(description='AirSim docker image builder')
    parser.add_argument('--source', action='store_true', help='compile unreal and airsim from source')
    parser.add_argument('--base_image', type=str, help='base image name AND tag')
    parser.add_argument('--target_image', type=str, help='target image name AND tag')

    args = parser.parse_args()
    build_docker_image(args)


def build_docker_image(args):
    # Context is the repository root so Dockerfiles can COPY PythonClient.
    docker_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(docker_dir)

    if args.source:
        dockerfile = os.path.join(docker_dir, 'Dockerfile_source')
        if not args.base_image:
            args.base_image = "ghcr.io/epicgames/unreal-engine:dev-slim-5.5.4"
        target_image_tag = args.base_image.split(":")[1]
        if not args.target_image:
            args.target_image = 'airsim_source' + ':' + target_image_tag
    else:
        dockerfile = os.path.join(docker_dir, 'Dockerfile_binary')
        if not args.base_image:
            args.base_image = "ghcr.io/epicgames/unreal-engine:dev-slim-5.5.4"
        target_image_tag = args.base_image.split(":")[1]
        if not args.target_image:
            args.target_image = 'airsim_binary' + ':' + target_image_tag

    docker_command = [
        'docker', 'build', '--network=host',
        '-t', args.target_image,
        '-f', dockerfile,
        '--build-arg', 'BASE_IMAGE=' + args.base_image,
        repo_root,
    ]
    print(" ".join(docker_command))
    subprocess.call(docker_command)


if __name__ == "__main__":
    main()
