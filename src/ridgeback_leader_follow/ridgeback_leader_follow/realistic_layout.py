"""Shared metre-scale layouts for the realistic visual scenario profile.

Keep static obstacle footprints outside the six fixtures' pedestrian lanes.
This module contains rendering inputs, not perception/identity inputs.
"""
from dataclasses import dataclass

import numpy as np

ENVIRONMENTS=('modern_office','office','warehouse','courtyard')


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
 # Modern office: textured surfaces keep their own pixels; flat finishes below.
 'laminate_floor_02':((.62,.5,.36),.55,0.),
 'white_oak_veneer':((.72,.6,.45),.5,0.),'marble_01':((.86,.84,.8),.25,0.),
 'denim_fabric_04':((.32,.38,.46),.9,0.),'aluminium':((.72,.73,.74),.35,.9),
 'matt_white':((.93,.93,.92),.6,0.),'wall_paint':((.88,.87,.84),.9,0.),'graphite':((.11,.115,.12),.5,.1),
 'felt':((.36,.38,.4),.95,0.),'wool_rug':((.66,.62,.56),.95,0.),'frost':((.95,.96,.97),.3,0.),
 'led':((.98,.96,.92),.4,0.),'ceiling_white':((.95,.95,.94),.7,0.),'screen':((.05,.07,.1),.15,0.),
}
# Finishes that emit light, in linear RGB, shared by both renderers.
# The ceiling glows faintly: it stands in for bounce light from the bright room.
EMISSIVE={'led':(3.,2.88,2.65),'screen':(.18,.3,.55),'ceiling_white':(.16,.16,.16)}


def layout(environment):
    if environment not in ENVIRONMENTS:raise ValueError('Unknown environment: '+environment)
    boxes=[];props=[]
    def box(name,pos,size,material,collision=True,yaw=0.):
        boxes.append(Fixture(name,pos,size,material,collision,yaw))
    def prop(name,asset,x,y,z=0.,scale=1.,yaw=0.,collision=True):
        props.append(Prop(name,asset,(x,y,z),scale,yaw,collision))
    if environment=='modern_office':
        _modern_office(box,prop);return boxes,props
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


# Contemporary open-plan office around the observer at the origin facing +X.
# Pedestrian lanes occupy x 0.5..21.5, |y| <= 4.2; everything solid stays out.
CEILING=3.0
X0,X1,Y0,Y1=-3.6,24.,-7.5,7.6


def _modern_office(box,prop):
    H=CEILING;mid=(X0+X1)/2;length=X1-X0
    box('Ground',(mid,0,-.12),(length+1,Y1-Y0+1,.24),'laminate_floor_02')
    box('Terrace',(mid,-12.5,-.13),(length+12,9,.24),'concrete_floor_02')
    box('Ceiling',(mid,0,H+.08),(length+1,Y1-Y0+1,.16),'ceiling_white')
    box('NorthWall',(mid,Y1+.1,H/2),(length+.4,.2,H),'wall_paint')
    box('WestWall',(X0-.1,0,H/2),(.2,Y1-Y0+.4,H),'wall_paint')
    box('EastWall',(X1+.1,0,H/2),(.2,Y1-Y0+.4,H),'wall_paint')
    for name,pos,size in (('SkirtNorth',(mid,Y1-.01,.04),(length,.02,.08)),('SkirtWest',(X0+.01,0,.04),(.02,Y1-Y0,.08)),
                          ('SkirtEast',(X1-.01,0,.04),(.02,Y1-Y0,.08))):
        box(name,pos,size,'graphite',False)
    # South curtain wall: full-height glazing between slim aluminium mullions.
    box('FacadeHead',(mid,Y0,H-.06),(length,.14,.12),'aluminium')
    box('FacadeSill',(mid,Y0,.05),(length,.14,.1),'aluminium')
    for i,x in enumerate(np.arange(X0,X1+.01,1.5)):
        box(f'FacadeMullion{i}',(float(x),Y0,H/2),(.06,.14,H),'aluminium')
        if x+1.5<=X1+.01:box(f'FacadeGlass{i}',(float(x)+.75,Y0,H/2),(1.44,.02,H-.2),'glass',False)
    box('FacadeTransom',(mid,Y0+.01,2.35),(length,.1,.05),'aluminium',False)
    # Linear LED ceiling lights in runs along the room.
    for row,y in enumerate((-5.6,-2.,2.)):
        for i,x in enumerate(np.arange(-1.,22.,4.5)):
            box(f'LEDRun{row}_{i}',(float(x),y,H-.015),(3.6,.09,.03),'led',False)
    # Workstations facing the window: oak bench desks, white frames, monitors.
    for c,x in enumerate((2.,5.8,9.6,13.4,17.2)):
        box(f'DeskTop{c}',(x,-6.45,.725),(3.2,.8,.03),'white_oak_veneer')
        for j,dx in enumerate((-1.55,0.,1.55)):
            box(f'DeskFrame{c}_{j}',(x+dx,-6.45,.355),(.04,.72,.71),'matt_white')
        box(f'DeskScreen{c}',(x,-6.83,.92),(3.2,.025,.36),'felt',False)
        for s,dx in enumerate((-.8,.8)):
            px=x+dx
            box(f'Monitor{c}_{s}',(px,-6.6,1.06),(.62,.025,.37),'graphite',False)
            box(f'Display{c}_{s}',(px,-6.585,1.065),(.585,.006,.335),'screen',False)
            box(f'MonitorNeck{c}_{s}',(px,-6.62,.82),(.04,.04,.16),'aluminium',False)
            box(f'MonitorFoot{c}_{s}',(px,-6.6,.745),(.22,.16,.012),'aluminium',False)
            box(f'Keyboard{c}_{s}',(px,-6.25,.747),(.44,.14,.016),'graphite',False)
            _task_chair(box,f'Chair{c}_{s}',px+(.05 if s else -.05),-5.6)
        prop(f'DeskPlant{c}','potted_plant_04',x+.1,-6.7,.74,scale=1.)
        if c%2:prop(f'DeskBinder{c}','binder_notebook',x-.15,-6.5,.74,scale=.8,collision=False)
    for i,x in enumerate((3.9,11.5,19.1)):
        prop(f'FloorPlant{i}','potted_plant_02',x,-5.2,scale=1.35)
    # Glazed meeting rooms along the north wall.
    yp=4.75
    for k,(xa,xb) in enumerate(((6.,13.),(13.,20.))):
        for j,x in enumerate(np.arange(xa,xb+.01,1.4)):
            box(f'PartitionPost{k}_{j}',(float(x),yp,H/2),(.05,.08,H),'aluminium')
            if x+1.4<=xb+.01:
                door=j==1
                box(f'PartitionGlass{k}_{j}',(float(x)+.7,yp,H/2),(1.34,.012,H),'glass',False)
                box(f'Manifestation{k}_{j}',(float(x)+.7,yp-.012,1.15),(1.34,.004,.12),'frost',False)
                if door:box(f'DoorPull{k}',(float(x)+1.25,yp-.04,1.05),(.025,.03,1.1),'aluminium',False)
        box(f'PartitionHead{k}',((xa+xb)/2,yp,H-.04),(xb-xa,.09,.08),'aluminium')
        box(f'RoomWall{k}',(xa,(yp+Y1)/2,H/2),(.12,Y1-yp,H),'wall_paint')
        cx=(xa+xb)/2
        box(f'MeetingTable{k}',(cx,6.15,.725),(2.8 if k==0 else 2.2,1.05,.04),'white_oak_veneer')
        for j,dx in enumerate((-.9,.9) if k==0 else (-.6,.6)):
            box(f'TableLeg{k}_{j}',(cx+dx,6.15,.355),(.08,.6,.71),'matt_white')
        seats=(-1.,0.,1.) if k==0 else (-.55,.55)
        for j,dx in enumerate(seats):
            _task_chair(box,f'MeetChairS{k}_{j}',cx+dx,5.35,front=90.)
            _task_chair(box,f'MeetChairN{k}_{j}',cx+dx,6.95,front=-90.)
        box(f'WallDisplay{k}',(xb-.08,6.15,1.55),(.03,1.45,.82),'graphite',False)
        box(f'WallDisplayGlow{k}',(xb-.1,6.15,1.55),(.006,1.4,.77),'screen',False)
        prop(f'Pendant{k}','modern_ceiling_lamp_01',cx,6.15,H-.95)
        box(f'RoomLED{k}',(cx,6.9,H-.015),(xb-xa-1.,.09,.03),'led',False)
    box('RoomWallEast',(20.,(yp+Y1)/2,H/2),(.12,Y1-yp,H),'wall_paint')
    # Lounge: block sofa, mid-century chairs, rug, shelving and planting.
    box('LoungeRug',(1.2,6.05,.006),(4.,2.6,.012),'wool_rug',False)
    _sofa(box,'Sofa',1.2,7.05)
    for i,x in enumerate((-.1,2.5)):
        prop(f'LoungeChair{i}','mid_century_lounge_chair',x,5.25,yaw=180.)
    prop('LoungeTable','coffee_table_round_01',1.2,6.15,scale=.62)
    prop('LoungeSide','side_table_01',3.8,7.0)
    for i,(x,y) in enumerate(((-3.,7.),(4.9,7.05),(-3.,-6.6),(23.4,-6.8),(23.4,6.9))):
        prop(f'TallPlant{i}','potted_plant_02',x,y,scale=1.5)
    # Cube shelving (3 x 4 cubbies, 0.36 m) dressed with books and succulents.
    for i,y in enumerate((3.,5.)):
        prop(f'Shelving{i}','wooden_display_shelves_01',X0+.2,y)
        for j,(dy,z,kind) in enumerate(((-.36,.79,'books'),(.36,1.16,'books'),(0.,1.16,'plant'),(.36,.42,'plant'),(-.36,1.16,'books'))):
            if kind=='books':prop(f'ShelfBooks{i}_{j}','book_encyclopedia_set_01',X0+.2,y+dy,z,scale=.3,yaw=90.,collision=False)
            else:prop(f'ShelfPlant{i}_{j}','potted_plant_04',X0+.2,y+dy,z,collision=False)
    prop('Credenza','modern_wooden_cabinet',-.3,-7.05,scale=.85)
    # Reception: oak-slatted feature wall, marble-topped counter, waiting chairs.
    for i,y in enumerate(np.arange(-3.,3.01,.12)):
        box(f'Slat{i}',(X1-.04,float(y),H/2),(.05,.045,H-.02),'white_oak_veneer',False)
    box('ReceptionSign',(X1-.075,0.,1.95),(.025,2.6,.82),'sign_reception',False)
    box('ReceptionCounter',(22.3,4.3,.53),(.7,2.6,1.06),'white_oak_veneer')
    box('ReceptionTop',(22.3,4.3,1.08),(.78,2.68,.04),'marble_01')
    box('ReceptionDesk',(23.2,4.3,.36),(.75,2.4,.04),'white_oak_veneer',False)
    box('ReceptionDisplay',(23.,4.,1.06),(.03,.5,.3),'graphite',False)
    for i,y in enumerate((-4.9,-6.0)):
        prop(f'WaitingChair{i}','modern_arm_chair_01',22.6,y,yaw=90.)
    prop('WaitingTable','side_table_01',22.7,-5.45,scale=.9)


def _task_chair(box,name,x,y,front=-90.):
    """Mesh-back task chair from boxes; `front` is the yaw the sitter faces."""
    import math
    a=math.radians(front);fx,fy=math.cos(a),math.sin(a)
    box(name+'Seat',(x,y,.47),(.48,.48,.07),'felt',yaw=front)
    box(name+'Back',(x-fx*.25,y-fy*.25,.8),(.05,.46,.5) if abs(fx)>.5 else (.46,.05,.5),'graphite',False)
    box(name+'Column',(x,y,.27),(.05,.05,.36),'aluminium',False)
    for k in range(5):
        box(name+f'Spoke{k}',(x,y,.06),(.6,.04,.03),'graphite',False,yaw=k*36.)


def _sofa(box,name,x,y):
    """Low three-seat fabric sofa against the wall behind it (its back at +y)."""
    box(name+'Base',(x,y,.2),(2.4,.92,.4),'denim_fabric_04')
    for i,dx in enumerate((-.73,0.,.73)):
        box(name+f'Cushion{i}',(x+dx,y-.08,.46),(.71,.74,.13),'denim_fabric_04',False)
        box(name+f'Pillow{i}',(x+dx,y+.32,.68),(.71,.2,.42),'denim_fabric_04',False)
    for i,dx in enumerate((-1.13,1.13)):
        box(name+f'Arm{i}',(x+dx,y,.32),(.14,.92,.64),'denim_fabric_04',False)
    for i,(dx,dy) in enumerate(((-1.1,-.38),(1.1,-.38),(-1.1,.38),(1.1,.38))):
        box(name+f'Foot{i}',(x+dx,y+dy,.02),(.05,.05,.04),'graphite',False)
