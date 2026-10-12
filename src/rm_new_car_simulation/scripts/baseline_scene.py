#!/usr/bin/env python3
"""Build reproducible SDF / URDF / map artifacts without a STEP reader.

2025 is a native reference scene. 2026 is a dimensioned fixture course, not a
complete 2026 field. Every export shares the centered world frame and metres.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from scipy.spatial import ConvexHull

NAMES = ['front_left', 'front_right', 'rear_left', 'rear_right']
DY = np.array([-1., 1., 1., -1.])


def load_config(root):
    return (yaml.safe_load((root/'config/new_car_cad_baseline.yaml').read_text()),
            yaml.safe_load((root/'config/baseline.yaml').read_text()))


def wheel_matrix(cad):
    w = cad['wheel']
    x = np.array([1,1,-1,-1]) * w['wheelbase_m']/2
    y = np.array([1,-1,1,-1]) * w['track_m']/2
    return np.column_stack([np.ones(4), DY, x*DY-y]) / (math.sqrt(2)*w['radius_m'])


def sensor_geometry(cad, cfg, selection):
    center = np.array([*cad['gimbal']['yaw_axis_xy_m'],cfg['gimbal_origin_ground_z_m']])
    home = np.array(cad['mid360'][selection]['cad_model_origin_xyz_m'],float)
    home[2] += cad['wheel']['radius_m']
    return center, home-center


def collision_footprint(cad,cfg,selection):
    """Enclose every collision proxy, including omni contact spheres and yaw sweep.

    CAD eight-point body outline does not contain the supplied wheel centers.
    This explicit conflict must not be hidden by using a smaller Nav2 polygon.
    """
    points=[tuple(p) for p in cad['footprint_m']]
    radius=cad['wheel']['radius_m']
    def circle(center,r):
        # Circumscribed 32-gon: every edge stays outside the true circle.
        rr=r/math.cos(math.pi/32)
        points.extend((center[0]+rr*math.cos(2*math.pi*i/32),
                       center[1]+rr*math.sin(2*math.pi*i/32)) for i in range(32))
    for i in range(4):
        xy=[cad['wheel']['wheelbase_m']/2*(1 if i<2 else -1),
            cad['wheel']['track_m']/2*(1 if i in [0,2] else -1)]
        circle(xy,radius)
    center,offset=sensor_geometry(cad,cfg,selection)
    circle(center[:2],max(.2,max(np.linalg.norm(offset[:2]+[x,y]) for x in [-.04,.04] for y in [-.04,.04])))
    def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    ordered=sorted(set(points));lower=[];upper=[]
    for p in ordered:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    for p in reversed(ordered):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return [list(p) for p in lower[:-1]+upper[:-1]]


def navigation_footprint(cad,cfg,selection,profile='deformed'):
    """Safety margin belongs to navigation, never to mechanical collision."""
    from scipy.spatial.transform import Rotation
    roll_limit,pitch_limit=cfg['navigation_attitude_domain_deg']
    if roll_limit<0 or pitch_limit<0:raise ValueError('attitude domain must be nonnegative')
    center,offset=sensor_geometry(cad,cfg,selection)
    r=cad['wheel']['radius_m'];points3=[]
    for z in [cfg['lower_collision_z_m'],cfg['upper_envelope_top_m'][profile]]:
        points3.extend([*p,z] for p in cad['footprint_m'])
    angles=np.arange(32)*2*math.pi/32
    for z,rr in [(center[2],.2),(center[2]+.04,.2),
                 (center[2]+offset[2]-.03,max(np.linalg.norm(offset[:2]+[x,y]) for x in [-.04,.04] for y in [-.04,.04])),
                 (center[2]+offset[2]+.03,max(np.linalg.norm(offset[:2]+[x,y]) for x in [-.04,.04] for y in [-.04,.04]))]:
        rr/=math.cos(math.pi/32)
        points3.extend([center[0]+rr*math.cos(a),center[1]+rr*math.sin(a),z] for a in angles)
    physical=[];max_radius=float(np.linalg.norm(points3,axis=1).max())
    rolls=np.linspace(-roll_limit,roll_limit,max(1,int(math.ceil(roll_limit*4))+1))
    pitches=np.linspace(-pitch_limit,pitch_limit,max(1,int(math.ceil(pitch_limit*2))+1))
    travel=.03 if cfg.get('wheel_support')=='compliant_proxy' else 0.
    wheel_centers=np.array([[cad['wheel']['wheelbase_m']/2*x,cad['wheel']['track_m']/2*y,r+q]
        for x,y in [(1,1),(1,-1),(-1,1),(-1,-1)] for q in [-travel,travel]])
    max_radius=max(max_radius,float(np.linalg.norm(wheel_centers,axis=1).max()+r))
    wheel_circle=np.column_stack([np.cos(angles),np.sin(angles)])*r/math.cos(math.pi/32)
    for roll in rolls:
        for pitch in pitches:
            rot=Rotation.from_euler('xyz',[roll,pitch,0],degrees=True)
            physical.extend(rot.apply(points3)[:,:2])
            physical.extend((rot.apply(wheel_centers)[:,:2,None]+wheel_circle.T).transpose(0,2,1).reshape(-1,2))
    physical=np.asarray(physical);physical=physical[ConvexHull(physical).vertices]
    # Cover the continuous attitudes between samples using the maximum point
    # displacement bound. This is numerical enclosure, not extra mechanical size.
    half_step=(np.max(np.diff(rolls))/2 if len(rolls)>1 else 0)+(np.max(np.diff(pitches))/2 if len(pitches)>1 else 0)
    sampling_reserve=2*max_radius*math.sin(math.radians(half_step)/2)
    margin=cfg['navigation_safety_margin_m']
    if margin<0: raise ValueError('navigation safety margin must be nonnegative')
    margin+=sampling_reserve
    if margin==0:return physical.tolist()
    offsets=np.column_stack([np.cos(angles),np.sin(angles)])*margin/math.cos(math.pi/32)
    points=(physical[:,None,:]+offsets).reshape(-1,2)
    return points[ConvexHull(points).vertices].tolist()


def vector(v):
    return ' '.join(f'{float(x):.12g}' for x in v)


def prism(points, height):
    return '<polyline><height>'+str(height)+'</height>'+''.join(
        '<point>'+vector(p)+'</point>' for p in points)+'</polyline>'


def body_geometry(geom, pose='0 0 0 0 0 0', name='body', color='0.12 0.25 0.65 1'):
    return (f'<collision name="{name}_collision"><pose>{pose}</pose><geometry>{geom}</geometry>'
            '</collision>'+f'<visual name="{name}_visual"><pose>{pose}</pose><geometry>{geom}</geometry>'
            f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>')


def inertial(mass, moments, position=(0,0,0)):
    return f'<inertial><pose>{vector(position)} 0 0 0</pose><mass>{mass}</mass><inertia><ixx>{moments[0]}</ixx><iyy>{moments[1]}</iyy>'\
           f'<izz>{moments[2]}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'


def robot_sdf(cad, cfg, selection, profile, sensors):
    radius = cad['wheel']['radius_m']
    center, offset = sensor_geometry(cad,cfg,selection)
    poly = cad['footprint_m']
    upper = cfg['upper_envelope_top_m'][profile]
    parts = ['<model name="new_car"><pose>0 0 0.002 0 0 0</pose>',
             '<link name="base_link">'+inertial(cfg['mass_kg'], cfg['inertia_kg_m2'],cfg['base_inertial_proxy_xyz_m'])]
    parts += [body_geometry(prism(poly,.115-cfg['lower_collision_z_m']),
                f'0 0 {cfg["lower_collision_z_m"]} 0 0 0', 'chassis'),
              body_geometry(prism(poly,upper-.115),'0 0 0.115 0 0 0','upper_proxy'),'</link>']
    parts += [f'<link name="gimbal_yaw_link"><pose relative_to="base_link">{vector(center)} 0 0 0</pose>',
              inertial(.2,[.0002]*3),body_geometry('<cylinder><radius>0.20</radius><length>0.04</length></cylinder>',
                 '0 0 0.02 0 0 0','gimbal'),'</link>',
              '<joint name="gimbal_yaw_joint" type="revolute"><parent>base_link</parent><child>gimbal_yaw_link</child>'
              '<axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis></joint>',
              f'<link name="sim_lidar_link"><pose relative_to="gimbal_yaw_link">{vector(offset)} {vector(cfg["sensor_rpy_rad"])}</pose>',
              inertial(.1,[.0001]*3),body_geometry('<box><size>0.08 0.08 0.06</size></box>',name='lidar',color='.1 .1 .1 1')]
    if sensors:
        parts += [f'''<sensor name="mid360_geometry_proxy" type="gpu_lidar"><topic>/model/new_car/lidar</topic><gz_frame_id>sim_lidar_link</gz_frame_id>
        <update_rate>{cfg['lidar_rate_hz']}</update_rate><always_on>true</always_on><lidar><scan><horizontal>
        <samples>{cfg['lidar_samples'][0]}</samples><resolution>1</resolution><min_angle>-3.14159265359</min_angle>
        <max_angle>3.14159265359</max_angle></horizontal><vertical><samples>{cfg['lidar_samples'][1]}</samples>
        <resolution>1</resolution><min_angle>-0.122173</min_angle><max_angle>0.907571</max_angle></vertical></scan>
        <range><min>0.1</min><max>30</max><resolution>0.01</resolution></range></lidar></sensor>''']
    parts += ['</link><joint name="lidar_mount_joint" type="fixed"><parent>gimbal_yaw_link</parent><child>sim_lidar_link</child></joint>']
    for i,name in enumerate(NAMES):
        xy = [cad['wheel']['wheelbase_m']/2 * (1 if i<2 else -1),
              cad['wheel']['track_m']/2 * (1 if i in [0,2] else -1)]
        # n = z cross rolling tangent; positive joint speed rolls in (+1,dy)/sqrt(2).
        axis = [-DY[i]/math.sqrt(2),1/math.sqrt(2),0]
        rolling = [1/math.sqrt(2),DY[i]/math.sqrt(2),0]
        compliant=cfg.get('wheel_support','rigid')=='compliant_proxy'
        parent='base_link';wheel_pose=vector([*xy,radius])
        if compliant:
            parent=name+'_carrier';wheel_pose='0 0 0'
            stiffness=cfg['support_stiffness_n_m'];damping=cfg['support_damping_ns_m']
            # Numerical contact regularization: preload supports the declared
            # proxy mass at the nominal CAD wheel-center height, never a claimed
            # real suspension stiffness, travel, rest position or load balance.
            preload=-(cfg['mass_kg']+4*cfg['wheel_mass_kg']+.34)*9.8/(4*stiffness)
            parts += [f'<link name="{parent}"><pose relative_to="base_link">{vector([*xy,radius])} 0 0 0</pose>'+
                inertial(.01,[1e-6]*3)+'</link>',
                f'<joint name="{name}_support_joint" type="prismatic"><parent>base_link</parent><child>{parent}</child>'
                '<axis><xyz expressed_in="base_link">0 0 1</xyz><limit><lower>-.03</lower><upper>.03</upper></limit>'
                f'<dynamics><damping>{damping}</damping><spring_reference>{preload}</spring_reference>'
                f'<spring_stiffness>{stiffness}</spring_stiffness></dynamics></axis></joint>']
        if cfg.get('friction_basis','legacy_rolling_tangent')=='axle_contact_normal':
            # DART projects fdir1 onto the contact plane; its second tangent is
            # n_contact cross fdir1. Give the axle as the free-roller direction
            # so the driven tangent follows the actual contact normal at crests.
            fdir,mu1,mu2=axis,cfg['roller_friction'],cfg['rolling_friction']
        elif cfg.get('friction_basis','legacy_rolling_tangent')=='legacy_rolling_tangent':
            fdir,mu1,mu2=rolling,cfg['rolling_friction'],cfg['roller_friction']
        else:raise ValueError('unknown friction basis')
        contact_sensor=(f'<sensor name="{name}_contact" type="contact"><always_on>true</always_on>'
            f'<update_rate>50</update_rate><contact><topic>/model/new_car/contact/{name}</topic>'
            '<collision>ideal_roller_contact</collision></contact></sensor>') if cfg.get('contact_audit') else ''
        parts += [f'<link name="{name}_wheel"><pose relative_to="{parent}">{wheel_pose} 0 0 0</pose>',
          inertial(cfg['wheel_mass_kg'],[.4*cfg['wheel_mass_kg']*radius**2]*3),
          f'''<collision name="ideal_roller_contact"><geometry><sphere><radius>{radius}</radius></sphere></geometry>
          <surface><friction><ode><mu>{mu1}</mu><mu2>{mu2}</mu2>
          <fdir1 gz:expressed_in="base_link">{vector(fdir)}</fdir1></ode></friction></surface></collision>
          <visual name="wheel"><geometry><sphere><radius>{radius}</radius></sphere></geometry></visual>{contact_sensor}</link>
          <joint name="{name}_wheel_joint" type="revolute"><parent>{parent}</parent><child>{name}_wheel</child>
          <axis><xyz expressed_in="base_link">{vector(axis)}</xyz></axis></joint>''']
    parts += [f'''<plugin filename="librm_omni_wheel_drive.so" name="rm_sim::OmniWheelDrive">
      <wheel_radius>{radius}</wheel_radius><wheelbase>{cad['wheel']['wheelbase_m']}</wheelbase>
      <track>{cad['wheel']['track_m']}</track><command_timeout>{cfg['command_timeout_sim_sec']}</command_timeout></plugin></model>''']
    return ''.join(parts)


def robot_urdf(cad,cfg,selection,profile,visual_mesh):
    center,offset = sensor_geometry(cad,cfg,selection)
    parts=[f'<robot name="new_car"><link name="base_link"><visual><geometry><mesh filename="{visual_mesh}"/></geometry></visual></link>',
           '<link name="cad_wheel_center"/><joint name="cad_reference" type="fixed"><parent link="base_link"/>'
           f'<child link="cad_wheel_center"/><origin xyz="0 0 {cad["wheel"]["radius_m"]}"/></joint>',
           '<link name="gimbal_yaw_link"><visual><origin xyz="0 0 .02"/><geometry><cylinder radius=".20" length=".04"/></geometry></visual></link>'
           '<joint name="gimbal_yaw_joint" type="continuous"><parent link="base_link"/>'
           f'<child link="gimbal_yaw_link"/><origin xyz="{vector(center)}"/><axis xyz="0 0 1"/></joint>',
           '<link name="sim_lidar_link"><visual><geometry><box size=".08 .08 .06"/></geometry></visual></link>'
           '<joint name="lidar_mount_joint" type="fixed"><parent link="gimbal_yaw_link"/>'
           f'<child link="sim_lidar_link"/><origin xyz="{vector(offset)}" rpy="{vector(cfg["sensor_rpy_rad"])}"/></joint>']
    for i,name in enumerate(NAMES):
        xyz = [cad['wheel']['wheelbase_m']/2 * (1 if i<2 else -1),
               cad['wheel']['track_m']/2 * (1 if i in [0,2] else -1),cad['wheel']['radius_m']]
        parent='base_link'
        if cfg.get('wheel_support','rigid')=='compliant_proxy':
            parent=name+'_carrier'
            parts += [f'<link name="{parent}"/><joint name="{name}_support_joint" type="prismatic">'
                f'<parent link="base_link"/><child link="{parent}"/><origin xyz="{vector(xyz)}"/>'
                '<axis xyz="0 0 1"/><limit lower="-.03" upper=".03" effort="10000" velocity="10"/></joint>']
            xyz=[0,0,0]
        parts += [f'<link name="{name}_wheel"><visual><geometry><sphere radius="{cad["wheel"]["radius_m"]}"/></geometry></visual></link>'
            f'<joint name="{name}_wheel_joint" type="continuous">'
            f'<parent link="{parent}"/><child link="{name}_wheel"/><origin xyz="{vector(xyz)}"/>'
            f'<axis xyz="{vector([-DY[i]/math.sqrt(2),1/math.sqrt(2),0])}"/></joint>']
    return ''.join(parts)+'</robot>'


def box(name, xyz, size, rpy=(0,0,0), color='.45 .45 .45 1'):
    return body_geometry('<box><size>'+vector(size)+'</size></box>',vector([*xyz,*rpy]),name,color)


def fixture_scene(dog_slope=False,section='B',tolerance='nominal'):
    if section not in ['A','B']:raise ValueError('unknown tunnel section')
    # Placements are test-layout coordinates, not a reconstruction of Figure 4-5.
    parts = [box('north',[0,7.55,1.2],[28.2,.1,2.4]),box('south',[0,-7.55,1.2],[28.2,.1,2.4]),
             box('west',[-14.05,0,1.2],[.1,15,2.4]),box('east',[14.05,0,1.2],[.1,15,2.4])]
    # Fig 4-35. A uses the conservative 700 mm restriction through the throat;
    # its exact tapered section is not fabricated from the undimensioned drawing.
    # Throat depth, side height and wall thickness remain fixture assumptions.
    width,height=(.7,.26) if section=='A' else (.8,.25)
    if tolerance=='lower_clearance':width*=.95;height*=.95
    elif tolerance!='nominal':raise ValueError('unknown field tolerance profile')
    floor=.06 if dog_slope else 0.
    parts += [box('tunnel_left',[0,4-width/2-.05,.25+floor],[.35,.1,.5]),
              box('tunnel_right',[0,4+width/2+.05,.25+floor],[.35,.1,.5]),
              box('tunnel_roof',[0,4,height+.025+floor],[.35,width+.2,.05])]
    if dog_slope:
        parts += [box('tunnel_floor',[0,4,floor/2],[.35,width,floor])]
        for side,angle in [(-1,11),(1,15)]:
            pitch=side*math.radians(angle);length=floor/math.tan(math.radians(angle))
            x=side*(.175+length/2)-.02*math.sin(pitch)
            parts += [box('tunnel_ramp_'+str(side),[x,4,floor/2-.02*math.cos(pitch)],
                      [length/math.cos(pitch),width,.04],(0,pitch,0))]
    for name,x,y,angle,height,width in [('road11',-6,0,11,.3,1.8),
                                      ('road15',-6,-4,15,.3,1.8),('central10_5',5,0,10.5,.35,2.972)]:
        length=height/math.tan(math.radians(angle))
        # Exact top-surface end elevations 0 and height, thickness 0.04 normal to slope.
        z=height/2-.02*math.cos(math.radians(angle))
        parts += [box(name,[x+.02*math.sin(math.radians(angle)),y,z],[length/math.cos(math.radians(angle)),width,.04],
                      (0,-math.radians(angle),0),color='.3 .5 .7 1'),
                  box(name+'_deck',[x+length/2+1,y,height/2],[2,width,height])]
    parts += [box('narrow_top',[7,4.45,.4],[4,.1,.8]),box('narrow_bottom',[7,3.55,.4],[4,.1,.8])]
    return '<model name="rules2026_course"><static>true</static><link name="field">'+''.join(parts)+'</link></model>'


def read_stl(path):
    data=path.read_bytes(); count=struct.unpack('<I',data[80:84])[0]
    if len(data)!=84+count*50: raise ValueError('expected binary STL')
    a=np.frombuffer(data,dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')]),offset=84,count=count)
    return a['v'].astype(float)


def clip_navigation_height(triangles,ceiling=2.4):
    """Clip the source's 18.55 m exterior walls/overhead detail to navigation height.

    Preserve every low obstacle / ramp / opening; never use a whole-scene convex hull.
    The native floor top is z=0 (two 232 m2 floor triangles), not z=-0.2.
    """
    output=[]
    for t in triangles:
        if t[:,2].max()<=ceiling: output.append(t);continue
        p=[]
        for a,b in zip(t,np.roll(t,-1,axis=0)):
            if a[2]<=ceiling: p.append(a)
            if (a[2]<=ceiling)!=(b[2]<=ceiling):
                p.append(a+(b-a)*(ceiling-a[2])/(b[2]-a[2]))
        for i in range(1,len(p)-1): output.append(np.array([p[0],p[i],p[i+1]]))
    return np.array(output)


def write_stl(path,triangles):
    a=np.zeros(len(triangles),dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')]))
    a['v']=triangles
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    length=np.linalg.norm(normals,axis=1);valid=length>1e-12
    a['normal'][valid]=normals[valid]/length[valid,None]
    path.write_bytes(b'RM navigation-height excerpt; native XY and floor z=0'.ljust(80,b' ')+struct.pack('<I',len(a))+a.tobytes())


def surface_points(triangles, spacing=.12):
    # Uniform barycentric sampling with spatial deduplication. The PCD is a synthetic
    # mesh-surface map, not a scan/bag or an occupancy-to-PCD fabrication.
    if not len(triangles): return np.empty((0,3))
    points=[]
    for t in triangles:
        steps=max(1,int(math.ceil(max(np.linalg.norm(t[1]-t[0]),np.linalg.norm(t[2]-t[0]))/spacing)))
        for i in range(steps+1):
            j=np.arange(steps-i+1)
            points.extend(t[0]+(i*(t[1]-t[0])+j[:,None]*(t[2]-t[0]))/steps)
    p=np.asarray(points)
    _,indices=np.unique(np.round(p/0.03).astype(np.int32),axis=0,return_index=True)
    return p[np.sort(indices)]


def primitive_triangles(scene):
    triangles=[]
    faces=[[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],
           [2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]]
    for c in ET.fromstring(scene).findall('.//collision'):
        pose=list(map(float,c.findtext('pose').split()))
        size=np.array(list(map(float,c.findtext('geometry/box/size').split())))
        r,p,y=pose[3:];cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
        rot=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                      [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
        v=np.array([[x,y,z] for z in [-.5,.5] for y in [-.5,.5] for x in [-.5,.5]])*size
        v=v@rot.T+pose[:3]
        triangles.extend(v[faces])
    return np.array(triangles)


def export_maps(out,triangles,cfg,revision):
    p=surface_points(triangles)
    with (out/'field.pcd').open('wb') as f:
        f.write(('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                 f'WIDTH {len(p)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(p)}\nDATA binary\n').encode())
        f.write(p.astype('<f4').tobytes())
    res=cfg['map_resolution_m'];origin=[-15.,-8.5,0.];nx,ny=600,340
    occupied=np.zeros((ny,nx),bool)
    # Conservative full projection: elevated roads/decks are occupied as well.
    # It never claims multi-level traversability or opens an uncertified underpass.
    for t in triangles:
        if t[:,2].max()<.04 or t[:,2].min()>2.5: continue
        mn=np.ceil((t[:,:2].min(axis=0)-origin[:2])/res).astype(int)-1
        mx=np.floor((t[:,:2].max(axis=0)-origin[:2])/res).astype(int)
        xs=np.arange(max(0,mn[0]),min(nx-1,mx[0])+1);ys=np.arange(max(0,mn[1]),min(ny-1,mx[1])+1)
        if not len(xs) or not len(ys): continue
        xx,yy=np.meshgrid(origin[0]+(xs+.5)*res,origin[1]+(ys+.5)*res)
        # Triangle / closed occupancy-cell intersection by separating axes.
        # Unlike a barycentric tolerance, this has a metric meaning independent
        # of triangle size or tessellation and also handles vertical wall lines.
        vertices=t[:,:2];delta=np.roll(vertices,-1,axis=0)-vertices
        axes=np.vstack([np.eye(2),np.column_stack([-delta[:,1],delta[:,0]])])
        inside=np.ones_like(xx,dtype=bool)
        for axis in axes:
            if axis@axis<1e-20:continue
            projected=vertices@axis;centers=xx*axis[0]+yy*axis[1]
            half=res*.5*np.abs(axis).sum()
            inside &= (centers+half>=projected.min()-1e-10)&(centers-half<=projected.max()+1e-10)
        occupied[np.ix_(ys,xs)] |= inside
    img=np.where(occupied,0,254).astype(np.uint8)
    with (out/'field.pgm').open('wb') as f:
        f.write(f'P5\n{nx} {ny}\n255\n'.encode());f.write(np.flipud(img).tobytes())
    (out/'field.yaml').write_text(yaml.safe_dump(dict(image='field.pgm',mode='trinary',resolution=res,
        origin=origin,negate=0,occupied_thresh=.65,free_thresh=.25)))
    return {'revision':revision,'frame':'map','world_to_map':[0,0,0,0,0,0], 'units':'m',
            'origin':origin,'resolution_m':res,'size':[nx,ny],'point_count':len(p),
            'pcd_kind':'synthetic_surface_sampling','occupancy_kind':'conservative_cell_intersection_all_structure_projection',
            'multi_level_navigation_certified':False}


def generate(root,out,field='rules2026_course',selection='A',profile='deformed',sensors=False,spawn=None,physics_config=None,
             tunnel_section='B',field_tolerance='nominal',ground_bank_deg=1.5):
    cad,cfg=load_config(root);out.mkdir(parents=True,exist_ok=True)
    if physics_config:
        overrides=yaml.safe_load(Path(physics_config).read_text())
        allowed={'mass_kg','base_inertial_proxy_xyz_m','inertia_kg_m2',
                 'wheel_mass_kg','rolling_friction','roller_friction','contact_audit','friction_basis',
                 'wheel_support','support_stiffness_n_m','support_damping_ns_m'}
        if not isinstance(overrides,dict) or set(overrides)-allowed:
            raise ValueError('physics_config only accepts documented dynamics proxy parameters')
        cfg.update(overrides)
    for key in ['mass_kg','wheel_mass_kg']:
        if not math.isfinite(cfg[key]) or cfg[key]<=0:raise ValueError(f'{key} requires a finite positive proxy')
    for key in ['rolling_friction','roller_friction']:
        if not math.isfinite(cfg[key]) or cfg[key]<0:raise ValueError(f'{key} requires a finite nonnegative proxy')
    for key in ['inertia_kg_m2','base_inertial_proxy_xyz_m']:
        if len(cfg[key])!=3 or not np.isfinite(cfg[key]).all():raise ValueError(f'invalid {key}')
    if min(cfg['inertia_kg_m2'])<=0 or max(cfg['inertia_kg_m2'])>sum(cfg['inertia_kg_m2'])/2:
        raise ValueError('inertia proxy requires positive, physically consistent principal moments')
    if cfg.get('wheel_support','rigid') not in ['rigid','compliant_proxy']:
        raise ValueError('unknown wheel support model')
    if cfg.get('wheel_support')=='compliant_proxy' and not all(math.isfinite(cfg[k]) and cfg[k]>0
            for k in ['support_stiffness_n_m','support_damping_ns_m']):
        raise ValueError('compliant support requires positive numerical stiffness/damping')
    if field=='competition2025_reference':
        source=(root/'assets/polarbear_2025/rmuc_2025.stl').resolve()
        visual_triangles=clip_navigation_height(read_stl(source))
        mesh=(out/'field_navigation.stl').resolve();write_stl(mesh,visual_triangles)
        triangles=visual_triangles
        collision_mesh=mesh
        collision_audit=dict(method='retain_clipped_native_triangles',
            collision_triangles=len(triangles),whole_scene_convex_hull=False)
        scene='<model name="pb2025_reference"><static>true</static><link name="field">'+\
            f'<collision name="native_2025_collision"><geometry><mesh><uri>{collision_mesh}</uri></mesh></geometry></collision>'+\
            f'<visual name="native_2025_visual"><geometry><mesh><uri>{mesh}</uri></mesh></geometry></visual></link></model>'
        revision=cfg['competition_reference_revision']
        ground=''
    elif field=='banked_ground':
        if not 1<=ground_bank_deg<=2:raise ValueError('rules ground bank fixture requires 1..2 degrees')
        angle=math.radians(ground_bank_deg);parts=[]
        for side in [-1,1]:
            roll=-side*angle
            parts.append(box('bank_'+str(side),[0,side*4.25-.05*math.sin(side*angle),
                -4.25*math.tan(angle)-.05*math.cos(angle)],[30,8.5/math.cos(angle),.1],(roll,0,0)))
        scene='<model name="banked_ground"><static>true</static><link name="ground">'+''.join(parts)+'</link></model>'
        triangles=primitive_triangles(scene);ground='';revision=f'rmuc2026_ground_bank_fixture_{ground_bank_deg:g}deg_s2_v1'
    elif field in ['rules2026_course','dog_slope','flat']:
        scene=fixture_scene(field=='dog_slope',tunnel_section,field_tolerance) if field!='flat' else ''
        triangles=primitive_triangles(scene) if scene else np.empty((0,3,3))
        revision=(cfg['rules_fixture_revision']+'_'+tunnel_section+'_'+field_tolerance+
            ('_dog_slope_hypothesis' if field=='dog_slope' else '')) if scene else 'new_car_flat_v1'
        ground='<model name="ground"><static>true</static><link name="ground">'+\
            body_geometry('<box><size>30 17 .10</size></box>','0 0 -.05 0 0 0','ground')+'</link></model>'
    else: raise ValueError(field)
    robot=robot_sdf(cad,cfg,selection,profile,sensors)
    if spawn is not None:
        robot=robot.replace('<pose>0 0 0.002 0 0 0</pose>', '<pose>'+vector(spawn)+'</pose>',1)
    xml='<sdf version="1.9" xmlns:gz="http://gazebosim.org/schema"><world name="new_car_baseline">'+\
        '<physics name="baseline" type="ignored"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics>'
    for name in ['Physics','UserCommands','SceneBroadcaster']:
        filename={'Physics':'physics','UserCommands':'user-commands','SceneBroadcaster':'scene-broadcaster'}[name]
        xml+=f'<plugin filename="gz-sim-{filename}-system" name="gz::sim::systems::{name}"/>'
    if cfg.get('contact_audit'):
        xml+='<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>'
    if sensors: xml+='<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>'
    xml+='<light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><direction>-.5 .1 -1</direction><diffuse>.8 .8 .8 1</diffuse></light>'
    xml+=ground+scene+robot+'</world></sdf>'
    (out/'world.sdf').write_text(xml)
    poly=cad['footprint_m'];n=len(poly);obj=[]
    for z in [cfg['lower_collision_z_m'],cfg['upper_envelope_top_m'][profile]]:
        obj.extend('v '+vector([*point,z]) for point in poly)
    for i in range(n):
        j=(i+1)%n
        obj += [f'f {i+1} {j+1} {j+n+1} {i+n+1}']
    obj += ['f '+' '.join(str(i+1) for i in range(n-1,-1,-1)),
            'f '+' '.join(str(i+n+1) for i in range(n))]
    visual_mesh=(out/'chassis_proxy.obj').resolve();visual_mesh.write_text('\n'.join(obj)+'\n')
    (out/'robot.urdf').write_text(robot_urdf(cad,cfg,selection,profile,visual_mesh))
    manifest=export_maps(out,triangles,cfg,revision)
    center,offset=sensor_geometry(cad,cfg,selection)
    manifest.update(robot_revision=cfg['robot_revision'],field=field,lidar_selection=selection,
        base_link_reference=cfg['base_link'],contact_audit_enabled=bool(cfg.get('contact_audit',False)),
        geometry_profile=profile,gimbal_center_m=center.tolist(),sensor_offset_m=offset.tolist(),
        effective_top_m=max(cfg['upper_envelope_top_m'][profile],center[2]+offset[2]+.03),
        wheel_matrix_rad_s_per_twist=wheel_matrix(cad).tolist(),
        navigation_footprint_m=navigation_footprint(cad,cfg,selection,profile),
        collision_proxy_footprint_m=collision_footprint(cad,cfg,selection),
        cad_body_outline_m=cad['footprint_m'],mechanical_full_envelope_m=None,
        mechanical_envelope_certified=False,
        navigation_safety_margin_m=cfg['navigation_safety_margin_m'],
        navigation_attitude_domain_deg=cfg['navigation_attitude_domain_deg'],
        navigation_footprint_kind='collision_proxy_gimbal_and_attitude_sweep_plus_navigation_margin',
        tunnel_section=tunnel_section,field_tolerance=field_tolerance,
        ground_bank_fixture_deg=ground_bank_deg if field=='banked_ground' else None,
        cad_body_polygon_contains_all_wheel_centers=False,
        spawn_world_xyz_rpy=spawn if spawn is not None else [0,0,.002,0,0,0],
        full_2026_field_certified=False,calibrated_dynamics=False)
    manifest['physics_proxy']={key:cfg[key] for key in ['mass_kg','base_inertial_proxy_xyz_m',
        'inertia_kg_m2','wheel_mass_kg','rolling_friction','roller_friction']}
    manifest['physics_proxy']['friction_basis']=cfg.get('friction_basis','legacy_rolling_tangent')
    manifest['physics_proxy']['wheel_support']=cfg.get('wheel_support','rigid')
    if cfg.get('wheel_support')=='compliant_proxy':
        manifest['physics_proxy'].update(support_stiffness_n_m=cfg['support_stiffness_n_m'],
            support_damping_ns_m=cfg['support_damping_ns_m'],support_travel_m=[-.03,.03],
            support_is_mechanical_calibration=False)
    if field=='competition2025_reference':manifest['collision_simplification']=collision_audit
    manifest['sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir()
                        if p.name in ['world.sdf','robot.urdf','field.pcd','field.pgm','field.yaml','field_navigation.stl','field_collision.stl','chassis_proxy.obj']}
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return manifest


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--field',choices=['flat','rules2026_course','dog_slope','banked_ground','competition2025_reference'],default='rules2026_course')
    p.add_argument('--lidar',choices=['A','B'],default='A');p.add_argument('--profile',choices=['deformed','undeformed'],default='deformed')
    p.add_argument('--sensors',action='store_true')
    p.add_argument('--physics-config',type=Path);p.add_argument('--tunnel-section',choices=['A','B'],default='B')
    p.add_argument('--field-tolerance',choices=['nominal','lower_clearance'],default='nominal')
    p.add_argument('--ground-bank-deg',type=float,default=1.5)
    args=p.parse_args();print(json.dumps(generate(args.root,args.output,args.field,args.lidar,args.profile,args.sensors,
        physics_config=args.physics_config,tunnel_section=args.tunnel_section,field_tolerance=args.field_tolerance,
        ground_bank_deg=args.ground_bank_deg),indent=2))
