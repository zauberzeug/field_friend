#!/usr/bin/env python3
import argparse

from livesync import Folder, sync

parser = argparse.ArgumentParser(description='Sync local code with robot.')
parser.add_argument('robot', help='Robot hostname')

args = parser.parse_args()
touch = 'touch ~/field_friend/main.py'
sync(
    Folder('.', f'{args.robot}:~/field_friend', on_change=touch),
    # Folder('../rosys/rosys', f'{args.robot}:~/field_friend/rosys', on_change=touch),
    # Folder('../nicegui/nicegui', f'{args.robot}:~/field_friend/nicegui', on_change=touch),
    # Folder('../lizard', f'{args.robot}:~/lizard', on_change=touch),
)
