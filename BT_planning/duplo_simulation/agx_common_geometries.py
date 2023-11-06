# pylint: disable=too-many-arguments
"""
Common geometries for agx simulation
"""
import agx
import agxOSG
import agxCollide
import agxRender

def add_table(sim, root, height, length, width, material, visual):
    """ Just a flat table """
    table = agx.RigidBody()
    table.setMotionControl(agx.RigidBody.STATIC)
    table.setName("table")
    table_geom = agxCollide.Geometry(agxCollide.Box(length / 2, width / 2, height / 2))
    table_geom.setName("table")
    table.add(table_geom)
    table_geom.setPosition(0, 0, -height / 2)
    table_geom.setMaterial(material)
    sim.add(table)

    if visual:
        m = agxOSG.createVisual(table, root)
        agxOSG.setDiffuseColor(m, agxRender.Color.SaddleBrown())

    return table
