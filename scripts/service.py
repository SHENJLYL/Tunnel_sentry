#!/usr/bin/env python3
"""List or run a configured workspace service in the current terminal."""
import argparse
import os
from pathlib import Path
import shlex
import yaml

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser()
parser.add_argument('name',nargs='?')
args=parser.parse_args()
config=yaml.safe_load((root/'ros2_services.yaml').read_text())
services=config['workspaces']['tunnel_sentinel_ws']['services']
if args.name is None:
    for item in services: print(item['unit_name']+'  '+item['description'])
else:
    selected=next((s for s in services if s['unit_name']==args.name),None)
    if selected is None: parser.error('Choose a listed service name.')
    os.chdir(root)
    os.execvp('bash',['bash','-c','source scripts/env.sh && exec '+shlex.join(shlex.split(selected['launch_command']))])
