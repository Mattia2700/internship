import argparse
import subprocess

def __main__():

    parser = argparse.ArgumentParser(description='remove running or stopped containers')
    args = parser.parse_args()
    subprocess.run(['docker', 'compose', 'down'])

if __name__ == '__main__':
    __main__()