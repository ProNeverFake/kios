# pylint: disable=too-many-instance-attributes, too-many-arguments, line-too-long
"""
Functions for creating duplo bricks in agx simulation
"""
from dataclasses import dataclass
from enum import Enum, auto
import math

import agx
import agxSDK
import agxOSG
import agxCollide

class BrickTypes(Enum):
    """ Enum of brick types """
    STANDARD2X2 = auto()
    HIGH2X2 = auto()
    STANDARD4X2 = auto()
    FAST4X2 = auto()
    CONE4X2 = auto()
    SPECIAL8X2PLUS2X2 = auto()

@dataclass
class BrickProps:
    """
    Properties of duplo bricks
    Default values are for a 2x2 medium height brick
    """
    fit_margin: float = 0.001                                            #Margin for fitting studs in holes
    mass: float = 0.006                                                  #Total mass of brick
    height: float = 0.0192                                               #Height of brick excluding studs
    length: float = 0.031                                                #Length of brick
    width: float = 0.031                                                 #Width of brick
    stud_height: float = 0.0034                                          #Total height of studs
    stud_radius: float = 0.0048                                          #Radius of each stud
    stud_centerdist: float = 0.008                                       #Distance from studcenter to brickcenter in x and y direction
    studtop_height: float = 0.001                                        #Height of guiding chamfer on top of stud
    studtop_inner_rad: float = stud_radius - studtop_height              #Inner radius of chamfer
    studcyl_height: float = stud_height - studtop_height                 #Height of cylinder part of stud only
    center_radius: float = \
        math.sqrt(2 * (stud_centerdist**2)) - stud_radius - fit_margin / 2 #Outer radius of hollow center cylinder
    wall_width: float = 0.0015                                           #Width of walls
    wall_height: float = height - wall_width                             #Height of walls
    center_height: float = wall_height - wall_width                      #Height of center cylinder
    ext_size: float = \
        width / 2 - wall_width - stud_centerdist - stud_radius - fit_margin / 2 #Size of extrudes for hole fit at bottom

    def update(self):
        """ Updates values of props that depends on other props. """
        self.studtop_inner_rad = self.stud_radius - self.studtop_height
        self.studcyl_height = self.stud_height - self.studtop_height
        self.center_radius = math.sqrt(2 * (self.stud_centerdist**2)) - self.stud_radius - self.fit_margin / 2
        self.wall_height = self.height - self.wall_width
        self.center_height = self.wall_height - self.wall_width
        self.ext_size = self.width / 2 - self. wall_width - self.stud_centerdist - self.stud_radius - self.fit_margin / 2

def rotate_x(geometry, degrees):
    """Rotate geometry around x axis """
    geometry.setLocalRotation(agx.EulerAngles(math.radians(degrees), 0, 0))

def rotate_z(geometry, degrees):
    """Rotate geometry around z axis """
    geometry.setLocalRotation(agx.EulerAngles(0, 0, math.radians(degrees)))

class Brick():
    """
    Generic duplo brick class
    """
    def __init__(self, props):
        self.brick_rb = agx.RigidBody()
        self.brick_rb.setName("duplobrick")
        self.height = props.height + props.stud_height #For broadcasting as sensor data
        self.props = props
        self.listeners = []
        self.hole_sensors = []

    def common_settings(self, sim, root, start_pos, visual, color):
        """
        Common settings used for all bricks but that must be done after init
        """
        self.brick_rb.getMassProperties().setMass(self.props.mass)
        self.brick_rb.setPosition(start_pos)

        self.set_amor_thresholds(1.e-3)

        sim.add(self.brick_rb)

        for listener in self.listeners:
            sim.add(listener)

        if visual:
            n = agxOSG.createVisual(self.brick_rb, root)
            agxOSG.setDiffuseColor(n, color)

    def set_amor_thresholds(self, threshold):
        """ Set amor thresholds of brick rigid body """
        properties = agxSDK.MergeSplitHandler.getOrCreateProperties(self.brick_rb)
        properties.setEnableMergeSplit(True)
        properties.getOrCreateContactThresholds().setSplitOnLogicalImpact(True)
        properties.getOrCreateContactThresholds().setMaxRelativeNormalSpeed(threshold)
        properties.getOrCreateContactThresholds().setMaxRelativeTangentSpeed(threshold)
        properties.getOrCreateContactThresholds().setMaxRollingSpeed(threshold)
        properties.getOrCreateContactThresholds().setMaxImpactSpeed(threshold)

    def add_top(self, props, material, x=0, y=0, z=0):
        """ Add top plate of brick """
        top = agxCollide.Geometry(agxCollide.Box(props.length / 2, props.width / 2, props.wall_width / 2))
        self.brick_rb.add(top)
        top.setLocalPosition(x, y, z + props.height - props.wall_width / 2)
        top.setMaterial(material)

    def add_walls(self, props, material, x=0, y=0, z=0):
        """ Add four walls """
        length_wall_1 = agxCollide.Geometry(agxCollide.Box(props.length / 2, props.wall_width / 2, props.wall_height / 2))
        length_wall_1.setMaterial(material)
        length_wall_2 = length_wall_1.clone()

        width_wall_1 = agxCollide.Geometry(agxCollide.Box(props.width / 2, props.wall_width / 2, props.wall_height / 2))
        width_wall_1.setMaterial(material)
        width_wall_2 = width_wall_1.clone()

        self.brick_rb.add(width_wall_1)
        self.brick_rb.add(width_wall_2)
        self.brick_rb.add(length_wall_1)
        self.brick_rb.add(length_wall_2)

        length_wall_1.setLocalPosition(x, y + props.width / 2 - props.wall_width / 2, z + props.wall_height / 2)
        length_wall_2.setLocalPosition(x, y - props.width / 2 + props.wall_width / 2, z + props.wall_height / 2)

        rotate_z(width_wall_1, 90)
        rotate_z(width_wall_2, 90)
        width_wall_1.setLocalPosition(x + props.length / 2 - props.wall_width / 2, y, z + props.wall_height / 2)
        width_wall_2.setLocalPosition(x - props.length / 2 + props.wall_width / 2, y, z + props.wall_height / 2)

    def add_center(self, props, material, x=0, y=0, z=0):
        """ Add hollow center cylinder """
        center = agxCollide.Geometry(agxCollide.HollowCylinder(props.center_radius - props.wall_width, props.center_height, props.wall_width))
        center.setMaterial(material)
        self.brick_rb.add(center)
        rotate_x(center, 90)
        center.setLocalPosition(x, y, z + props.wall_height / 2)

    def add_extrudes(self, props, material):
        """ Adds extrudes for hole fit at bottom wall_width """
        extrude1 = agxCollide.Geometry(agxCollide.Box(props.ext_size / 2, props.wall_width / 2, props.center_height / 2))
        extrude1.setMaterial(material)
        extrude2 = extrude1.clone()
        extrude3 = extrude1.clone()
        extrude4 = extrude1.clone()
        extrude5 = extrude1.clone()
        extrude6 = extrude1.clone()
        extrude7 = extrude1.clone()
        extrude8 = extrude1.clone()

        self.brick_rb.add(extrude1)
        self.brick_rb.add(extrude2)
        self.brick_rb.add(extrude3)
        self.brick_rb.add(extrude4)
        self.brick_rb.add(extrude5)
        self.brick_rb.add(extrude6)
        self.brick_rb.add(extrude7)
        self.brick_rb.add(extrude8)

        extrude1.setLocalPosition(props.length / 2 - props.wall_width - props.ext_size / 2, props.stud_centerdist, props.wall_height/2)
        extrude2.setLocalPosition(props.length / 2 - props.wall_width - props.ext_size / 2, -props.stud_centerdist, props.wall_height/2)
        rotate_z(extrude3, 90)
        extrude3.setLocalPosition(props.stud_centerdist, -(props.width / 2 - props.wall_width - props.ext_size / 2), props.wall_height/2)
        rotate_z(extrude4, 90)
        extrude4.setLocalPosition(-props.stud_centerdist, -(props.width / 2 - props.wall_width - props.ext_size / 2), props.wall_height/2)
        extrude5.setLocalPosition(-(props.length / 2 - props.wall_width - props.ext_size / 2), -props.stud_centerdist, props.wall_height/2)
        extrude6.setLocalPosition(-(props.length / 2 - props.wall_width - props.ext_size / 2), props.stud_centerdist, props.wall_height/2)
        rotate_z(extrude7, 90)
        extrude7.setLocalPosition(-props.stud_centerdist, props.width / 2 - props.wall_width - props.ext_size / 2, props.wall_height/2)
        rotate_z(extrude8, 90)
        extrude8.setLocalPosition(props.stud_centerdist, props.width / 2 - props.wall_width - props.ext_size / 2, props.wall_height/2)

        if props.length >= 2 * props.width:
            #Long brick, add more extrudes
            extrude9 = extrude3.clone()
            extrude10 = extrude3.clone()
            extrude11 = extrude3.clone()
            extrude12 = extrude3.clone()
            self.brick_rb.add(extrude9)
            self.brick_rb.add(extrude10)
            self.brick_rb.add(extrude11)
            self.brick_rb.add(extrude12)

            extrude9.setLocalPosition(props.stud_centerdist + props.width / 2, -(props.width / 2 - props.wall_width - props.ext_size / 2), props.wall_height/2)
            extrude10.setLocalPosition(-(props.stud_centerdist + props.width / 2), -(props.width / 2 - props.wall_width - props.ext_size / 2), props.wall_height/2)
            extrude11.setLocalPosition(-(props.stud_centerdist + props.width / 2), props.width / 2 - props.wall_width - props.ext_size / 2, props.wall_height/2)
            extrude12.setLocalPosition(props.stud_centerdist + props.width / 2, props.width / 2 - props.wall_width - props.ext_size / 2, props.wall_height/2)

    def add_stud(self, props, material, x, y, z=0):
        """ Adds stud at (x, y) """
        base = agxCollide.Geometry(agxCollide.Cylinder(props.stud_radius, props.studcyl_height))
        base.setMaterial(material)
        rotate_x(base, 90)
        top = agxCollide.Geometry(agxCollide.Cone(props.studtop_inner_rad, props.stud_radius, props.studtop_height))
        top.setMaterial(material)
        rotate_x(top, 90)

        self.brick_rb.add(base)
        self.brick_rb.add(top)

        base.setLocalPosition(x, y, z + props.height + props.studcyl_height / 2)
        top.setLocalPosition(x, y, z + props.height + props.studcyl_height)

    def add_stud_sensor(self, props, x, y):
        """ Adds sensor at stud for peg in hole simulation """
        stud_sensor = agxCollide.Geometry(agxCollide.Sphere(props.stud_radius))
        stud_sensor.addGroup("stud")
        stud_sensor.setSensor(True)
        self.brick_rb.add(stud_sensor)
        stud_sensor.setLocalPosition(x, y, props.height + props.studcyl_height - props.stud_radius)

    def add_hole_sensor(self, props, x, y, listener_pool):
        """
        Create a sensor geometry spere to represent the hole.
        When a stud sensor overlaps with this sphere
        a prismatic joint is created to simulate the holes friction
        """
        hole_sensor = agxCollide.Geometry(agxCollide.Sphere(props.stud_radius))
        hole_sensor.setSensor(True)
        self.brick_rb.add(hole_sensor)
        hole_sensor.setLocalPosition(x, y, props.stud_radius)
        self.hole_sensors.append(hole_sensor)

        contact_listener = listener_pool.get_listener()
        contact_listener.hole_depth = props.studcyl_height
        contact_listener.setFilter(agxSDK.GeometryFilter(hole_sensor))
        contact_listener.setEnable(True)
        self.listeners.append(contact_listener)

    def add_studs(self, props, material, center_x, center_y, listener_pool):
        """ Adds four studs around a center point at (center_x, center_y) """
        self.add_stud(props, material, center_x + props.stud_centerdist, center_y + props.stud_centerdist)
        self.add_stud(props, material, center_x + props.stud_centerdist, center_y - props.stud_centerdist)
        self.add_stud(props, material, center_x - props.stud_centerdist, center_y + props.stud_centerdist)
        self.add_stud(props, material, center_x - props.stud_centerdist, center_y - props.stud_centerdist)

        self.add_stud_sensor(props, center_x + props.stud_centerdist, center_y + props.stud_centerdist)
        self.add_stud_sensor(props, center_x + props.stud_centerdist, center_y - props.stud_centerdist)
        self.add_stud_sensor(props, center_x - props.stud_centerdist, center_y + props.stud_centerdist)
        self.add_stud_sensor(props, center_x - props.stud_centerdist, center_y - props.stud_centerdist)

        self.add_hole_sensor(props, center_x + props.stud_centerdist, center_y + props.stud_centerdist, listener_pool)
        self.add_hole_sensor(props, center_x + props.stud_centerdist, center_y - props.stud_centerdist, listener_pool)
        self.add_hole_sensor(props, center_x - props.stud_centerdist, center_y + props.stud_centerdist, listener_pool)
        self.add_hole_sensor(props, center_x - props.stud_centerdist, center_y - props.stud_centerdist, listener_pool)

    def remove_hole_collisions(self, rigid_body):
        """
        This functions disables collisions between the bricks holes
        and any geometry that is not a stud sensor
        """
        for geometry in rigid_body.getGeometries():
            if not geometry.hasGroup("stud"):
                for hole_sensor in self.hole_sensors:
                    hole_sensor.setEnableCollisions(geometry, False)

class Duplo4x2(Brick):
    """ 4x2 brick of standard height """
    def __init__(self, material, listener_pool):
        props = BrickProps()
        props.length *= 2
        props.mass *= 2
        super(Duplo4x2, self).__init__(props)

        self.add_top(props, material)
        self.add_walls(props, material)
        self.add_center(props, material)
        self.add_center(props, material, x=props.stud_centerdist * 2)
        self.add_center(props, material, x=-props.stud_centerdist * 2)
        self.add_extrudes(props, material)
        self.add_studs(props, material, props.stud_centerdist * 2, 0, listener_pool)
        self.add_studs(props, material, -props.stud_centerdist * 2, 0, listener_pool)

        self.brick_rb.setRotation(agx.EulerAngles(0, 0, math.radians(-90)))

class Duplo4x2Fast(Brick):
    """
    4x2 brick of standard height but only four listeners in the center studs.
    This gives a much faster and more reliable simulation when the end studs are not needed
    """
    def __init__(self, material, listener_pool):
        props = BrickProps()
        props.length *= 2
        props.mass *= 2
        super(Duplo4x2Fast, self).__init__(props)

        self.add_top(props, material)
        self.add_walls(props, material)
        self.add_center(props, material)
        self.add_center(props, material, x=props.stud_centerdist * 2)
        self.add_center(props, material, x=-props.stud_centerdist * 2)
        self.add_extrudes(props, material)
        self.add_studs(props, material, 0, 0, listener_pool)
        self.add_stud(props, material, 3 * props.stud_centerdist, props.stud_centerdist)
        self.add_stud(props, material, 3 * props.stud_centerdist, -props.stud_centerdist)
        self.add_stud(props, material, -3 * props.stud_centerdist, props.stud_centerdist)
        self.add_stud(props, material, -3 * props.stud_centerdist, -props.stud_centerdist)

        self.brick_rb.setRotation(agx.EulerAngles(0, 0, math.radians(-90)))

class Duplo4x2Cone(Brick):
    """ 4x2 brick of standard height with cone to mark and ensure that it can't be stacked """
    def __init__(self, material):
        props = BrickProps()
        props.length *= 2
        props.mass *= 3
        super(Duplo4x2Cone, self).__init__(props)

        self.add_top(props, material)
        self.add_walls(props, material)
        self.add_center(props, material)
        self.add_center(props, material, x=props.stud_centerdist * 2)
        self.add_center(props, material, x=-props.stud_centerdist * 2)
        self.add_extrudes(props, material)
        self.add_stud(props, material, 3 * props.stud_centerdist, props.stud_centerdist)
        self.add_stud(props, material, 3 * props.stud_centerdist, -props.stud_centerdist)
        self.add_stud(props, material, -3 * props.stud_centerdist, props.stud_centerdist)
        self.add_stud(props, material, -3 * props.stud_centerdist, -props.stud_centerdist)

        #Add cone
        base = agxCollide.Geometry(agxCollide.Box((props.width - 0.001) / 2, (props.width - 0.001) / 2, props.studcyl_height * 1.5))
        self.brick_rb.add(base)
        base.setLocalPosition(0, 0, props.height + props.studcyl_height * 1.5)
        base.setMaterial(material)

        cone_topradius = 2 * props.stud_radius
        cone_bottomradius = (props.width -0.001) / 2
        cone_height = 0.038
        cone = agxCollide.Geometry(agxCollide.Cone(cone_topradius, cone_bottomradius, cone_height))
        cone.setMaterial(material)
        rotate_x(cone, 90)
        self.brick_rb.add(cone)
        cone.setLocalPosition(0, 0, props.height)
        self.add_stud(props, material, x=0, y=0, z=cone_height)
        self.height += cone_height #Add cone height

        self.brick_rb.setRotation(agx.EulerAngles(0, 0, math.radians(-90)))

class Duplo2x2(Brick):
    """ 2x2 brick of standard height """
    def __init__(self, material, listener_pool):
        props = BrickProps()
        super(Duplo2x2, self).__init__(props)

        self.add_top(props, material)
        self.add_walls(props, material)
        self.add_center(props, material)
        self.add_extrudes(props, material)
        self.add_studs(props, material, 0, 0, listener_pool)

class Duplo2x2High(Brick):
    """ 2x2 brick of double height """
    def __init__(self, material, listener_pool):
        props = BrickProps()
        props.height *= 2
        props.mass *= 2
        props.update()
        super(Duplo2x2High, self).__init__(props)

        self.add_top(props, material)
        self.add_walls(props, material)
        self.add_center(props, material)
        self.add_extrudes(props, material)
        self.add_studs(props, material, 0, 0, listener_pool)

class Duplo8x2plus2x2(Brick):
    """ Low 8x2 brick with two 2x2 stacked as a stair and attached on the side """
    def __init__(self, material, listener_pool):
        props = BrickProps()
        props.height /= 2 #Height of lower part
        props.length *= 4
        props.mass *= 4
        props.update()
        super(Duplo8x2plus2x2, self).__init__(props)

        #Low 8x2 brick
        self.add_top(props, material)
        self.add_walls(props, material)
        self.add_center(props, material)
        self.add_center(props, material, x=props.stud_centerdist * 2)
        self.add_center(props, material, x=-props.stud_centerdist * 2)
        self.add_center(props, material, x=props.stud_centerdist * 4)
        self.add_center(props, material, x=-props.stud_centerdist * 4)
        self.add_center(props, material, x=props.stud_centerdist * 6)
        self.add_center(props, material, x=-props.stud_centerdist * 6)
        self.add_extrudes(props, material)
        self.add_studs(props, material, props.stud_centerdist * 2, 0, listener_pool)
        self.add_studs(props, material, -props.stud_centerdist * 2, 0, listener_pool)
        self.add_studs(props, material, -props.stud_centerdist * 6, 0, listener_pool)
        self.add_studs(props, material, props.stud_centerdist * 6, 0, listener_pool) #Needed to get mass disbribution correct

        #2x2 bricks
        props.height *= 2
        props.length /= 4
        props.update()
        x_offset = -props.length * 2
        z_offset = props.height / 4
        self.add_top(props, material, x=x_offset, y=0, z=z_offset)
        self.add_walls(props, material, x=x_offset, y=0, z=z_offset)
        self.add_center(props, material, x=x_offset, y=0, z=z_offset) #Needed to get mass disbribution correct
        self.add_stud(props, material, x_offset + props.stud_centerdist, props.stud_centerdist, z=z_offset)
        self.add_stud(props, material, x_offset + props.stud_centerdist, -props.stud_centerdist, z=z_offset)

        x_offset = -props.length * 2.5
        z_offset += props.height
        self.add_top(props, material, x=x_offset, y=0, z=z_offset)
        self.add_walls(props, material, x=x_offset, y=0, z=z_offset)
        self.add_center(props, material, x=x_offset, y=0, z=z_offset) #Needed to get mass disbribution correct

        self.add_stud(props, material, x_offset + props.stud_centerdist, props.stud_centerdist, z=z_offset)
        self.add_stud(props, material, x_offset + props.stud_centerdist, -props.stud_centerdist, z=z_offset)
        self.add_stud(props, material, x_offset - props.stud_centerdist, props.stud_centerdist, z=z_offset)
        self.add_stud(props, material, x_offset - props.stud_centerdist, -props.stud_centerdist, z=z_offset)

        self.brick_rb.setRotation(agx.EulerAngles(0, 0, math.radians(-90)))

def new_brick(sim, root, brick_type, start_pos, material, visual, color, listener_pool):
    """ Creates a new brick and returns it """
    brick = None
    if brick_type == BrickTypes.STANDARD2X2:
        brick = Duplo2x2(material, listener_pool)
    elif brick_type == BrickTypes.HIGH2X2:
        brick = Duplo2x2High(material, listener_pool)
    elif brick_type == BrickTypes.STANDARD4X2:
        brick = Duplo4x2(material, listener_pool)
    elif brick_type == BrickTypes.FAST4X2:
        brick = Duplo4x2Fast(material, listener_pool)
    elif brick_type == BrickTypes.CONE4X2:
        brick = Duplo4x2Cone(material)
    elif brick_type == BrickTypes.SPECIAL8X2PLUS2X2:
        brick = Duplo8x2plus2x2(material, listener_pool)
    brick.common_settings(sim, root, start_pos, visual, color)

    return brick
