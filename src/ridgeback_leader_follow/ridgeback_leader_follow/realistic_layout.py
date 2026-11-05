"""Shared metre-scale layouts for the realistic visual scenario profile.

Keep static obstacle footprints outside the six fixtures' pedestrian lanes.
This module contains rendering inputs, not perception/identity inputs.
"""
from dataclasses import dataclass

ENVIRONMENTS=('office','warehouse','courtyard')


@dataclass(frozen=True)
class Fixture:
    name: str
    position: tuple
    size: tuple
    material: str
    collision: bool=True
    yaw: float=0.


@dataclass(frozen=True)
class Prop:
    name: str
    asset: str
    position: tuple
    scale: float=1.
    yaw: float=0.
    collision: bool=True


PALETTE={
 'plaster':((.76,.74,.69),.85,0.),'ivory':((.88,.86,.8),.75,0.),
 'wood_table_001':((.38,.23,.11),.55,0.),'charcoal':((.045,.055,.065),.45,.35),
 'steel':((.24,.29,.32),.32,.8),'blue':((.05,.16,.24),.55,.15),
 'orange':((.75,.22,.025),.45,.25),'yellow':((.9,.58,.025),.6,0.),
 'glass':((.24,.46,.56),.08,.35),'white':((.9,.91,.89),.65,0.),
 'brick':((.4,.19,.1),.85,0.),
 **{name:((1.,1.,1.),.65,0.) for name in ('sign_reception','sign_office','sign_exit','sign_loading','sign_safety','sign_courtyard')},'wood_floor':((.5,.35,.18),.6,0.),
 'concrete_floor_02':((.5,.5,.48),.85,0.),'brick_pavement_02':((.5,.38,.3),.9,0.),
 'brick_wall_001':((.5,.35,.25),.85,0.),'grass_ground':((.18,.28,.07),.95,0.),
}


def layout(environment):
    if environment not in ENVIRONMENTS:raise ValueError('Unknown environment: '+environment)
    boxes=[];props=[]
    def box(name,pos,size,material,collision=True,yaw=0.):
        boxes.append(Fixture(name,pos,size,material,collision,yaw))
    def prop(name,asset,x,y,z=0.,scale=1.,yaw=0.,collision=True):
        props.append(Prop(name,asset,(x,y,z),scale,yaw,collision))
    floor={'office':'wood_floor','warehouse':'concrete_floor_02','courtyard':'brick_pavement_02'}[environment]
    box('Ground',(6,0,-.12),(26,18,.24),floor)
    if environment=='office':
        box('NorthWall',(6,6.5,2.1),(24,.22,4.2),'plaster')
        box('Ceiling',(6,0,4.2),(24,13,.16),'ivory')
        for side in (-1,1):box('CoveLED'+str(side).replace('-','n'),(6,side*5.5,4.0),(23,.06,.025),'white',False)
        box('EndBrickWall',(17,0,2.1),(.25,13,4.2),'brick_wall_001')
        box('WestWall',(-5,0,2.1),(.22,13,4.2),'plaster')
        box('EntranceLintel',(6,-6.5,3.8),(24,.22,.8),'plaster')
        for i,x in enumerate(range(-4,17,3)):
            box(f'Window{i}',(x,-6.5,1.9),(2.8,.06,3.25),'glass',False)
            box(f'WindowMullion{i}',(x-1.5,-6.5,1.9),(.07,.15,3.7),'charcoal')
            box(f'WindowSill{i}',(x,-6.5,.2),(3,.3,.08),'white')
        for y in (-6.3,6.3):
            box('Skirting'+str(y).replace('-','n').replace('.','_'),(6,y,.09),(23,.08,.18),'charcoal')
            box('CeilingStrip'+str(y).replace('-','n').replace('.','_'),(6,y,4.05),(23,1.2,.12),'ivory',False)
        for i,x in enumerate((0,5,10,15)):
            box(f'LightHousing{i}',(x,0,3.85),(2.4,.18,.08),'charcoal',False)
        for i,x in enumerate((0,6,12)):
            prop(f'LoungeSofa{i}','sofa_03',x,4.75,yaw=180)
            prop(f'LoungeChair{i}','modern_arm_chair_01',x+1.9,3.7,yaw=90)
            prop(f'LoungeTable{i}','modern_coffee_table_01',x,3.7)
            prop(f'LoungePlant{i}','potted_plant_02',x-1.8,4.2,scale=1.1)
        for i,x in enumerate((0,4.5)):
            prop(f'WorkDesk{i}','metal_office_desk',x,-4.8,yaw=180)
            prop(f'DeskChair{i}','modern_arm_chair_01',x,-3.65,yaw=180)
            box(f'Monitor{i}',(x,-4.8,1.12),(.65,.07,.4),'charcoal',False)
            box(f'MonitorStand{i}',(x,-4.8,.86),(.08,.15,.16),'steel',False)
            prop(f'DeskNotebook{i}','binder_notebook',x+.42,-4.75,.78,scale=.8)
            prop(f'DeskBooks{i}','book_encyclopedia_set_01',x-.5,-4.8,.78,scale=.5)
        box('ReceptionDesk',(11,-4.65,.6),(4,1.1,1.2),'wood_table_001')
        box('ReceptionCounter',(11,-4.65,1.24),(4.2,1.25,.08),'white')
        for i in range(24):box(f'ReceptionSlat{i}',(9.1+i*.165,-4.06,.6),(.07,.06,1.1),'charcoal',False)
        box('ReceptionBackdrop',(11,-6.32,2.1),(4.2,.1,2.8),'blue')
        box('ReceptionSign',(11,-6.22,2.5),(3.,.025,.95),'sign_reception',False)
        box('OfficeSign',(7,6.18,2.95),(2.5,.025,.78),'sign_office',False)
        box('ExitSign',(-4.8,0,2.6),(.025,1.7,.53),'sign_exit',False)
        prop('ReceptionNotebook','binder_notebook',11,-4.65,1.29,scale=.9)
        prop('ReceptionBin','metal_trash_can',13.5,-5.3,scale=.65)
        for i,x in enumerate((8,14)):
            prop(f'ReceptionPlant{i}','potted_plant_02',x,-4.6,scale=1.25)
        for i,x in enumerate((1,7,13)):
            box(f'RoomDoor{i}',(x,6.35,1.15),(1.12,.08,2.3),'wood_table_001')
            box(f'DoorFrame{i}',(x,6.4,1.18),(1.3,.1,2.46),'charcoal')
            box(f'DoorGlass{i}',(x,6.27,1.65),(.6,.02,.6),'glass',False)
            box(f'DoorHandle{i}',(x+.37,6.23,1.),(.04,.07,.22),'steel',False)
    elif environment=='warehouse':
        for side,y in enumerate((-7,7)):
            box(f'LowerWall{side}',(6,y,1.8),(24,.25,3.6),'brick_wall_001')
            box(f'UpperWall{side}',(6,y,5.2),(24,.25,.4),'plaster')
            for i,x in enumerate((-3,3,9,15)):
                for j,dx in enumerate((-2.35,2.35)):
                    box(f'ClerestoryJamb{side}_{i}_{j}',(x+dx,y,4.3),(.1,.2,1.4),'charcoal')
                for j,z in enumerate((3.65,4.95)):
                    box(f'ClerestoryRail{side}_{i}_{j}',(x,y,z),(4.8,.2,.1),'charcoal')
                box(f'ClerestoryGlass{side}_{i}',(x,y-side*.15+.075,4.3),(4.55,.04,1.18),'glass',False)
            for i,x in enumerate((-5.6,-.0,6.,12.,17.6)):
                box(f'WallPier{side}_{i}',(x,y,4.3),(1.2,.25,1.4),'brick_wall_001')
        box('LoadingEndWall',(17,0,2.7),(.25,14,5.4),'plaster')
        for side in (-1,1):box('Roof'+str(side).replace('-','n'),(6,side*4.65,5.5),(24,4.7,.16),'plaster')
        box('WarehouseSkylight',(6,0,5.5),(24,4.6,.08),'glass',False)
        box('LoadingDoor',(16.83,0,2.1),(.05,5,4.2),'steel')
        for i in range(22):box(f'DoorRib{i}',(16.75,0,.12+i*.185),(.04,5,.025),'charcoal',False)
        for side in (-1,1):
            for bay,x in enumerate((0,4,8,12)):
                y=side*4.8
                for dx in (-1.6,1.6):
                    for dy in (-.65,.65):box(f'RackPost{side}_{bay}_{dx}_{dy}'.replace('-','n').replace('.','_'),(x+dx,y+dy,1.95),(.1,.1,3.9),'blue')
                for level,z in enumerate((.15,1.45,2.8)):
                    box(f'Shelf{side}_{bay}_{level}'.replace('-','n'),(x,y,z),(3.3,1.45,.12),'orange')
                    for j,dx in enumerate((-.9,0,.9)):
                        asset='plastic_crate_01' if (bay+level+j)%3==0 else 'wooden_crate_01'
                        prop(f'Crate{side}_{bay}_{level}_{j}'.replace('-','n'),asset,x+dx,y,z+.07,scale=.7)
            for i,x in enumerate((2,6,10,14)):
                prop(f'Barrel{side}_{i}'.replace('-','n'),'barrel_03',x,side*3.3,scale=.85)
            for i in range(24):box(f'AislePaint{side}_{i}'.replace('-','n'),(-4+i*.9,side*2.65,.006),(.55,.07,.008),'yellow',False)
        for i,x in enumerate((-2,4,10,15)):
            box(f'CrossBeam{i}',(x,0,5.1),(.2,14,.2),'steel',False)
            box(f'LightHousing{i}',(x,0,4.9),(2,.25,.08),'white',False)
        prop('MaintenanceDesk','metal_office_desk',-3,5.5)
        box('LoadingSign',(16.7,0,4.6),(.025,3.3,1.),'sign_loading',False)
        box('PedestrianSign',(6,6.8,3.4),(3.,.025,.95),'sign_safety',False)
        prop('WarehouseBin','metal_trash_can',-3,3.8,scale=.85)
    else:
        for side in (-1,1):
            for i,x in enumerate((0,7,14)):
                box(f'GardenBed{side}_{i}'.replace('-','n'),(x,side*5.4,.18),(4.5,3,.36),'brick')
                box(f'GardenGrass{side}_{i}'.replace('-','n'),(x,side*5.4,.37),(4.3,2.8,.04),'grass_ground',False)
                prop(f'Tree{side}_{i}'.replace('-','n'),'tree_small_02',x,side*5.4,.4,scale=.65)
                for j,dx in enumerate((-1.25,1.25)):
                    for row,dy in enumerate((-.65,0.,.65)):
                        prop(f'Grass{side}_{i}_{j}_{row}'.replace('-','n'),'grass_medium_01',x+dx,side*(5.4+dy),.4,scale=.32,collision=False)
                prop(f'Shrub{side}_{i}'.replace('-','n'),'shrub_01',x,side*4.8,.4,scale=.4,collision=False)
                prop(f'Bench{side}_{i}'.replace('-','n'),'painted_wooden_bench',x,side*3.3,yaw=180 if side==1 else 0)
            for i,x in enumerate((2.5,10.5)):
                prop(f'StreetLamp{side}_{i}'.replace('-','n'),'street_lamp_01',x,side*3.9,scale=.8)
        box('NorthBuilding',(6,9,3.4),(26,.4,6.8),'brick_wall_001')
        box('EastBuilding',(19,0,3.4),(.4,18,6.8),'brick_wall_001')
        for row,z in enumerate((1.8,4.8)):
            for col,y in enumerate((-6,-3,0,3,6)):
                box(f'EastWindowFrame{row}_{col}',(18.75,y,z),(.1,1.5,1.9),'charcoal')
                box(f'EastWindow{row}_{col}',(18.68,y,z),(.03,1.3,1.7),'glass',False)
        box('EastCornice',(18.65,0,6.65),(.3,18,.25),'ivory')
        for row,z in enumerate((1.8,4.8)):
            for col,x in enumerate(range(-4,19,3)):
                box(f'WindowFrame{row}_{col}',(x,8.75,z),(1.5,.1,1.9),'charcoal')
                box(f'BuildingWindow{row}_{col}',(x,8.68,z),(1.3,.03,1.7),'glass',False)
                box(f'WindowSplit{row}_{col}',(x,8.62,z),(.04,.03,1.7),'white',False)
        box('CourtyardEntrance',(6,8.66,1.3),(2.5,.1,2.6),'charcoal')
        box('EntranceGlass',(6,8.58,1.3),(2.2,.04,2.4),'glass',False)
        box('CourtyardSign',(6,8.5,3.2),(3.5,.025,1.1),'sign_courtyard',False)
        for i,x in enumerate((3.2,10.2)):
            prop(f'CourtyardBin{i}','metal_trash_can',x,-3.7,scale=.7)
    return boxes,props
