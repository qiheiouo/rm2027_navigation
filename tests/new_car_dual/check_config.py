"""Launch/calibration negative controls. All numerical data here are synthetic."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import yaml
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[2]

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,ROOT/path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module

candidate=load('candidate','src/rm_lio_bringup/launch/new_car_dual_mid360_experimental.launch.py')
driver=load('driver','src/rm_mid360_driver_bridge/launch/dual_mid360_driver.launch.py')

def fixture():
    cfg=yaml.safe_load((ROOT/'src/rm_lio_bringup/config/new_car_dual_mid360_pending.yaml').read_text())
    m=cfg['/**']['ros__parameters']['mapping']
    for key in m:
        if key.startswith('extrinsic_T'):m[key]=[0.,0.,0.]
        if key.startswith('extrinsic_R'):m[key]=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
    # Synthetic common-IMU composition includes nonidentity rotation + translation.
    m['extrinsic_R2']=[-1.,0.,0.,0.,-1.,0.,0.,0.,1.]
    m['extrinsic_R_L2_wrt_L1']=m['extrinsic_R2'][:]
    m['extrinsic_T2']=[0.,-.24,0.];m['extrinsic_T_L2_wrt_L1']=m['extrinsic_T2'][:]
    return cfg

class ConfigTests(unittest.TestCase):
    def test_template_rejects_and_valid_rigid_composition_accepts(self):
        with self.assertRaises(RuntimeError):candidate.validate_config(ROOT/'src/rm_lio_bringup/config/new_car_dual_mid360_pending.yaml')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fixture.yaml';path.write_text(yaml.safe_dump(fixture()));candidate.validate_config(path)
    def test_reflection_nonfinite_contradiction_and_wrong_input_reject(self):
        for defect in ['reflection','nonfinite','contradiction','wrong_imu']:
            cfg=fixture();p=cfg['/**']['ros__parameters'];m=p['mapping']
            if defect=='reflection':m['extrinsic_R'][0]=-1.
            if defect=='nonfinite':m['extrinsic_T'][0]=float('nan')
            if defect=='contradiction':m['extrinsic_T_L2_wrt_L1'][0]=1.
            if defect=='wrong_imu':p['common']['imu_topic']='/right/imu'
            with tempfile.TemporaryDirectory() as directory:
                path=Path(directory)/'bad.yaml';path.write_text(yaml.safe_dump(cfg))
                with self.assertRaises(RuntimeError,msg=defect):candidate.validate_config(path)
    def test_default_zero_nodes_and_explicit_enable_requires_calibration(self):
        ctx=LaunchContext();ctx.launch_configurations['enable_experiment']='false'
        actions=candidate._launch_setup(ctx);self.assertTrue(all(isinstance(a,LogInfo) for a in actions))
        ctx.launch_configurations.update(enable_experiment='true',config_file='',calibration_revision='')
        with self.assertRaises(RuntimeError):candidate._launch_setup(ctx)
    def test_driver_network_override_and_one_sdk_two_adapters(self):
        with tempfile.TemporaryDirectory() as directory:
            cfg=json.loads((ROOT/'src/rm_mid360_driver_bridge/config/dual_mid360_config.json').read_text())
            # RFC5737 test-only identities; never production defaults.
            for entry,ip in zip(cfg['lidar_configs'],['192.0.2.10','192.0.2.11']):entry['ip']=ip
            path=Path(directory)/'test.json';path.write_text(json.dumps(cfg))
            ctx=LaunchContext();ctx.launch_configurations.update(use_driver='true',enable_left='true',enable_right='true',lio_imu_source='left',config_file=str(path),left_device_ip='192.0.2.10',right_device_ip='192.0.2.11',driver_package='livox_ros_driver2',left_frame_id='mid360_left_frame',right_frame_id='mid360_right_frame',publish_freq='50.0',output_type='0')
            actions=driver._launch_setup(ctx);self.assertEqual(sum(isinstance(a,Node) for a in actions),3)
            ctx.launch_configurations['lio_imu_source']='right';self.assertEqual(len(driver._launch_setup(ctx)),4)
            ctx.launch_configurations['right_device_ip']='192.0.2.10'
            with self.assertRaises(RuntimeError):driver._launch_setup(ctx)
            ctx.launch_configurations['right_device_ip']='192.0.2.12'
            with self.assertRaises(RuntimeError):driver._launch_setup(ctx)

if __name__=='__main__':unittest.main()
