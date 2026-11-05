"""Render the realistic worlds without ROS or a running interactive simulator.

Writes, per world and sampled scenario time: an overview, a human-height view,
a close cast view and the robot camera's view (its mount pose and 640x480
field of view), plus a USD snapshot of the last frame.
"""
import argparse
import math
from pathlib import Path
import sys

parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--environments',nargs='+',default=['modern_office'])
parser.add_argument('--scenario',default='crowd');parser.add_argument('--crowd-size',type=int,default=8)
parser.add_argument('--times',nargs='+',type=float,default=[2.])
args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
sys.argv=[sys.argv[0]]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':800,'renderer':'RaytracedLighting'})
# Robot camera: mount from clearpath/robot.yaml and the 640x480 colour profile.
ROBOT_EYE=(.27,0.,1.04);APERTURE=20.955
ROBOT_FOCAL=APERTURE/(2*math.tan(math.atan(320/384.)))
try:
    import omni.usd
    import omni.replicator.core as rep
    import numpy as np
    from PIL import Image
    from pxr import Gf,UsdGeom
    from ament_index_python.packages import get_package_share_directory
    from ridgeback_leader_follow.realistic_scene import create_scene,load_scenario
    from ridgeback_leader_follow.crowd_geometry import animate_isaac_person
    for environment in args.environments:
        omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage()
        scenario=load_scenario(args.scenario,args.crowd_size,environment);people=create_scene(stage,scenario,environment)
        adapter=Path(get_package_share_directory('ridgeback_autonomy_isaac'))/'sim/isaac'
        robot=stage.DefinePrim('/ridgeback','Xform');robot.GetReferences().AddReference((adapter/'usd/robots/ridgeback_r100/ridgeback_r100.usda').as_uri())
        UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(0,0,.06))
        if environment=='modern_office':
            views=[('overview',(-2.8,-3.6,2.55),(9.,.5,.9),16.,(1280,800)),('human_view',(1.2,-2.4,1.62),(10.,.8,1.25),18.,(1280,800)),
                   ('cast',(9.5,-2.2,1.6),(5.,.6,1.05),24.,(1280,800)),('robot_camera',ROBOT_EYE,(10.,0.,ROBOT_EYE[2]),ROBOT_FOCAL,(1280,960))]
        else:
            views=[('overview',(-3.5,-2.6,3.1),(7,0,1.),18.,(1280,800)),('human_view',(-1.8,-1.9,1.65),(5.5,0,1.),18.,(1280,800)),
                   ('cast',(8.,-2.5,1.9),(3.5,0,1.),18.,(1280,800)),('robot_camera',ROBOT_EYE,(10.,0.,ROBOT_EYE[2]),ROBOT_FOCAL,(1280,960))]
        for at in args.times:
            for person in scenario.people:animate_isaac_person(people[person.name],person,at)
            for name,eye,target,focal,resolution in views:
                camera=rep.create.camera(position=eye,look_at=target,focal_length=focal,horizontal_aperture=APERTURE)
                product=rep.create.render_product(camera,resolution)
                annotator=rep.AnnotatorRegistry.get_annotator('rgb');annotator.attach(product)
                for _ in range(32):app.update()
                rep.orchestrator.step(rt_subframes=16,pause_timeline=True)
                data=annotator.get_data()
                if data.size==0:raise RuntimeError('Empty render')
                stem=f'{environment}-{args.scenario}-t{at:04.1f}-{name}'
                Image.fromarray(np.asarray(data)[:,:,:3]).save(args.output/(stem+'.png'))
                annotator.detach(product);product.destroy()
                print('Rendered',stem,flush=True)
        stage.GetRootLayer().Export(str(args.output/(environment+'.usda')))
except BaseException:
    import traceback
    traceback.print_exc()
    app.close(exit_code=1)
    raise
finally:app.close()
