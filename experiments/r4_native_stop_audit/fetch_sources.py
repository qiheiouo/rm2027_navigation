#!/usr/bin/env python3
"""Read official pinned source references into an ignored audit cache, never vendor."""
import argparse
import hashlib
import json
import pathlib
import urllib.request

REFERENCES = {
    'gazebosim/gz-sim': ('ignition-gazebo6_6.18.0', 'Apache-2.0', [
        'src/systems/mecanum_drive/MecanumDrive.cc',
        'src/systems/odometry_publisher/OdometryPublisher.cc',
        'src/systems/physics/Physics.cc']),
    'gazebosim/gz-physics': ('ignition-physics5_5.4.0', 'Apache-2.0', [
        'dartsim/src/SDFFeatures.cc']),
    'ros-navigation/navigation2': ('1.1.20', 'Apache-2.0 (ControllerServer/VelocitySmoother); BSD-3-Clause (SimpleGoalChecker)', [
        'nav2_controller/plugins/simple_goal_checker.cpp',
        'nav2_controller/src/controller_server.cpp',
        'nav2_velocity_smoother/src/velocity_smoother.cpp']),
}


def get(url):
    with urllib.request.urlopen(urllib.request.Request(url,headers={'User-Agent':'rm2027-readonly-source-audit'}), timeout=30) as r:
        return r.read()


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=pathlib.Path);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=True);record=[]
    for repo,(tag,license,paths) in REFERENCES.items():
        revision=json.loads(get('https://api.github.com/repos/'+repo+'/commits/'+tag))['sha']
        for path in paths:
            url='https://raw.githubusercontent.com/'+repo+'/'+revision+'/'+path
            data=get(url);destination=args.output/(repo.replace('/','_')+'_'+pathlib.Path(path).name)
            destination.write_bytes(data)
            record.append(dict(repository='https://github.com/'+repo,tag=tag,commit=revision,path=path,
                license=license,url=url,sha256=hashlib.sha256(data).hexdigest(),cache_name=destination.name,
                use='read-only semantics audit; unmodified cache in ignored build only, no vendoring/runtime import'))
    (args.output/'sources.json').write_text(json.dumps(record,indent=2)+'\n')


if __name__=='__main__':main()
