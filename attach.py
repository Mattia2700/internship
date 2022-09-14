import argparse
import subprocess

def __main__():

    parser = argparse.ArgumentParser(description='attach to a desired container')
    exclusive = parser.add_mutually_exclusive_group()

    exclusive.add_argument('-r','--robot', help='attach to robot container', action='store_true')
    exclusive.add_argument('-b','--bridge', help='attach to ROS1-ROS2 bridge node', action='store_true')
    exclusive.add_argument('-g','--gazebo', help='attach to simulation container', action='store_true')

    exclusive.add_argument('-m','--map', help='attach to map container', action='store_true')
    
    exclusive.add_argument('-n','--navigation', help='attach to navigation container', action='store_true')
    exclusive.add_argument('-s','--slam', help='attach to slam container', action='store_true')

    exclusive.add_argument('-p','--planning', help='attach to planning container', action='store_true')
    exclusive.add_argument('-u','--ugv', help='attach to UGV actions container', action='store_true')
    exclusive.add_argument('-w','--webserver', help='attach to webserver container', action='store_true')

    args = parser.parse_args()
    if not any(vars(args).values()):
        parser.print_help()
        return

    service = [name for name in vars(args).keys() if vars(args)[name]][0]

    subprocess.run(['docker', 'exec', '-it', f'internship-{service}-1', '/bin/bash'])

if __name__ == '__main__':
    __main__()