"""Render the realistic worlds without ROS or a running interactive simulator."""
import argparse
from pathlib import Path
import sys

parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--environments',nargs='+',default=['office','warehouse','courtyard'])
args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
sys.argv=[sys.argv[0]]
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':800,'renderer':'RaytracedLighting'})
try:
    import omni.usd
    import omni.replicator.core as rep
    import numpy as np
    from PIL import Image
    from pxr import Gf,UsdGeom
    from ament_index_python.packages import get_package_share_directory
    from ridgeback_leader_follow.realistic_scene import create_scene
    from ridgeback_leader_follow.crowd_scenarios import load_scenario
    from ridgeback_leader_follow.crowd_geometry import animate_isaac_person
    for environment in args.environments:
        omni.usd.get_context().new_stage();stage=omni.usd.get_context().get_stage()
        scenario=load_scenario('crowd',8);people=create_scene(stage,scenario,environment)
        for person in scenario.people:animate_isaac_person(people[person.name],person,2.)
        adapter=Path(get_package_share_directory('ridgeback_autonomy_isaac'))/'sim/isaac'
        robot=stage.DefinePrim('/ridgeback','Xform');robot.GetReferences().AddReference((adapter/'usd/robots/ridgeback_r100/ridgeback_r100.usda').as_uri())
        UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(0,0,.06))
        views=[('overview',(-3.5,-2.6,3.1),(7,0,1.)),('human_view',(-1.8,-1.9,1.65),(5.5,0,1.)),('cast',(8.,-2.5,1.9),(3.5,0,1.))]
        for name,eye,target in views:
            camera=rep.create.camera(position=eye,look_at=target,focal_length=18.)
            product=rep.create.render_product(camera,(1280,800))
            annotator=rep.AnnotatorRegistry.get_annotator('rgb');annotator.attach(product)
            for _ in range(32):app.update()
            rep.orchestrator.step(rt_subframes=12,pause_timeline=True)
            data=annotator.get_data()
            if data.size==0:raise RuntimeError('Empty render')
            Image.fromarray(np.asarray(data)[:,:,:3]).save(args.output/(environment+'-'+name+'.png'))
            annotator.detach(product);product.destroy()
            print('Rendered',environment,name,flush=True)
        stage.GetRootLayer().Export(str(args.output/(environment+'.usda')))
except BaseException:
    import traceback
    traceback.print_exc()
    app.close(exit_code=1)
    raise
finally:app.close()
