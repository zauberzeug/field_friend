#!/usr/bin/env python3
import argparse

from livesync import Folder, sync
from livesync.run_subprocess import run_subprocess

parser = argparse.ArgumentParser(description='Sync local code with robot.')
parser.add_argument('robot', help='Robot hostname')
parser.add_argument('--rosys', action='store_true', default=False, help='Sync rosys')
parser.add_argument('--zedxmini', action='store_true', default=False, help='Sync zedxmini')

args = parser.parse_args()
touch = 'touch ~/field_friend/main.py'
folders = [Folder('.', f'{args.robot}:~/field_friend', on_change=touch)]
if args.rosys:
    folders.append(Folder('../rosys/rosys', f'{args.robot}:~/field_friend/rosys', on_change=touch))
else:
    print('Ensuring we have no local rosys on the robot')
    run_subprocess(f'ssh {args.robot} "rm -rf ~/field_friend/rosys"')

if args.zedxmini:
    folders.append(Folder('../zedxmini', f'{args.robot}:~/zedxmini', on_change=touch))
# else:
#     print('Ensuring we have no local zedxmini on the robot')
#     run_subprocess(f'ssh {args.robot} "rm -rf ~/zedxmini"')
sync(*folders)
