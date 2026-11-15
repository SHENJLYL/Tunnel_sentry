#!/usr/bin/env python3
"""Start a fresh simulation for each functional test and save the results."""
import os
import argparse
import xml.etree.ElementTree as E
from pathlib import Path
import signal
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
os.chdir(root)
os.environ['ROS_DOMAIN_ID']='48'
os.environ['GZ_PARTITION']='tunnel_sentinel_verify_48'
parser=argparse.ArgumentParser()
parser.add_argument('--case',choices=['full','collision','lidar'])
args=parser.parse_args()
for case in ([args.case] if args.case else ['full','collision','lidar']):
    directory=root/'reports'/case
    directory.mkdir(parents=True,exist_ok=True)
    command=['bash','scripts/run_sim.sh','gui:=false']
    if case=='lidar':
        tree=E.parse(root/'src/tunnel_sentinel_bringup/worlds/tunnel_test.sdf')
        world=tree.getroot().find('world')
        world.remove(world.find("model[@name='tunnel']"))
        wall=E.fromstring('<model name="lidar_wall"><static>true</static><pose>2 0 1 0 0 0</pose><link name="wall"><collision name="wall_collision"><geometry><box><size>.1 6 2</size></box></geometry></collision><visual name="wall_visual"><geometry><box><size>.1 6 2</size></box></geometry></visual></link></model>')
        world.append(wall)
        fixture=directory/'world.sdf';tree.write(fixture,encoding='utf-8',xml_declaration=True)
        command.append('world:='+str(fixture))
    with (directory/'simulation.log').open('w') as log:
        sim=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            result=subprocess.run([sys.executable,'scripts/demo.py','--case',case,'--output',str(directory)],timeout=240)
            if result.returncode: raise SystemExit(result.returncode)
        finally:
            if sim.poll() is None:
                sim.send_signal(signal.SIGINT)
                try: sim.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(sim.pid,signal.SIGTERM);sim.wait(timeout=8)
print('Requested simulation cases completed. Results: reports/<case>')
