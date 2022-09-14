import argparse
import yaml
import subprocess
import signal
from copy import deepcopy

def __main__():

    signal.signal(signal.SIGINT, signal.SIG_IGN)

    parser = argparse.ArgumentParser(description='start desired groups of nodes')
    
    parser.add_argument('-r','--robot', help='start robot nodes', action='store_true')
    parser.add_argument('-g','--gazebo', help='start simulation nodes', action='store_true')

    parser.add_argument('-n','--navigation', help='start navigation nodes', action='store_true')
    parser.add_argument('-s','--slam', help='start slam nodes', action='store_true')
    
    parser.add_argument('-p','--planning', help='start planning nodes', action='store_true')

    parser.add_argument('-d', '--detached', help='run containers in detached mode', action='store_true')

    args = parser.parse_args()
    if not any(vars(args).values()):
        parser.print_help()
        return

    if(args.robot and args.gazebo):
        print("Cannot start real and simulated nodes at the same time")
        exit(1)
    elif(args.navigation and args.slam):
        print("Cannot start navigation and slam nodes at the same time")
        exit(1)
    elif(args.planning and args.slam):
        print("Cannot start planning and slam nodes at the same time")
        exit(1)
    else:
        with open('template.yaml', 'r') as original:
            orig_yaml = yaml.full_load(original)
            edit_yaml = deepcopy(orig_yaml)

        services = orig_yaml['services']
        services_to_run = {name: services[name] for name in vars(args).keys() if vars(args)[name] and name!='detached'}

        if(args.robot):
            services_to_run['ros1-build'] = services['ros1-build']
            services_to_run['bridge'] = services['bridge']
        if(args.navigation or args.slam or args.gazebo):
            services_to_run['ros2-build-navigation'] = services['ros2-build-navigation']
        if(args.planning):
            services_to_run['ros2-build-planning'] = services['ros2-build-planning']
            services_to_run['ros2-build-navigation'] = services['ros2-build-navigation']
            services_to_run['ugv'] = services['ugv']
            services_to_run['webserver'] = services['webserver']

        edit_yaml['services'] = services_to_run

        with open('docker-compose.yaml', 'w') as real:
            yaml.dump(edit_yaml, real, indent=2)
        
        if(args.detached):
            subprocess.run(['docker', 'compose', 'up', '-d', '--remove-orphans']) 
        else:
            subprocess.run(['docker', 'compose', 'up', '--remove-orphans'])

if __name__ == '__main__':
    __main__()