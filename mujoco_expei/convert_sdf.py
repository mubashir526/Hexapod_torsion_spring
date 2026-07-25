import xml.etree.ElementTree as ET

tree = ET.parse('../ROS/src/sim_robot/models/THex_Quadruped/model.sdf')
root = tree.getroot()
model = root.find('model')

mjcf = ET.Element('mujoco', model="THex_Quadruped")

# Compiler options
compiler = ET.SubElement(mjcf, 'compiler', angle="radian", meshdir="meshes/")
option = ET.SubElement(mjcf, 'option', timestep="0.001", gravity="0 0 -9.81")
asset = ET.SubElement(mjcf, 'asset')

# Helper to get pose string
def get_pose(elem):
    p = elem.find('pose')
    return p.text if p is not None else "0 0 0 0 0 0"

# Parse all links and create meshes
links = {}
for link in model.findall('link'):
    name = link.get('name')
    pose = get_pose(link)
    
    # Inertial
    inert = link.find('inertial')
    mass = inert.find('mass').text if inert is not None else "1"
    pose_in = get_pose(inert) if inert is not None else "0 0 0 0 0 0"
    inertia = inert.find('inertia') if inert is not None else None
    if inertia is not None:
        ixx = inertia.find('ixx').text
        iyy = inertia.find('iyy').text
        izz = inertia.find('izz').text
        ixy = inertia.find('ixy').text
        ixz = inertia.find('ixz').text
        iyz = inertia.find('iyz').text
        diaginertia = f"{ixx} {iyy} {izz}" # MuJoCo uses full symmetric matrix or diag, we'll just pass full matrix below
        full_inertia = f"{ixx} {iyy} {izz} {ixy} {ixz} {iyz}"
    else:
        full_inertia = "1 1 1 0 0 0"
        
    # Visual
    vis = link.find('visual')
    mesh_uri = None
    if vis is not None:
        geom = vis.find('geometry')
        if geom is not None:
            mesh = geom.find('mesh')
            if mesh is not None:
                uri = mesh.find('uri').text
                # uri is like model://THex_Quadruped/meshes/base_link.STL
                mesh_uri = uri.split('/')[-1]
                # Add to asset
                if asset.find(f"mesh[@file='{mesh_uri}']") is None:
                    ET.SubElement(asset, 'mesh', file=mesh_uri)
    
    links[name] = {
        'pose': pose,
        'mass': mass,
        'inertial_pose': pose_in,
        'inertia': full_inertia,
        'mesh': mesh_uri
    }

# Parse joints
joints = {}
for joint in model.findall('joint'):
    name = joint.get('name')
    parent = joint.find('parent').text
    child = joint.find('child').text
    pose = get_pose(joint)
    axis = joint.find('axis')
    xyz = axis.find('xyz').text if axis is not None else "0 0 1"
    limit = axis.find('limit') if axis is not None else None
    lower = limit.find('lower').text if limit is not None else "-3.14"
    upper = limit.find('upper').text if limit is not None else "3.14"
    
    joints[child] = {
        'name': name,
        'parent': parent,
        'pose': pose,
        'axis': xyz,
        'range': f"{lower} {upper}"
    }

# Build tree starting from base_link
worldbody = ET.SubElement(mjcf, 'worldbody')
ET.SubElement(worldbody, 'light', diffuse=".5 .5 .5", pos="0 0 3", dir="0 0 -1")
# Add ground plane
ET.SubElement(worldbody, 'geom', type="plane", size="5 5 0.1", rgba=".9 .9 .9 1", material="")

def build_body(parent_elem, link_name):
    l = links[link_name]
    
    # Extract xyz and rpy
    parts = l['pose'].split()
    pos = " ".join(parts[:3])
    euler = " ".join(parts[3:])
    
    body = ET.SubElement(parent_elem, 'body', name=link_name, pos=pos, euler=euler)
    
    # Inertial
    i_parts = l['inertial_pose'].split()
    i_pos = " ".join(i_parts[:3])
    i_euler = " ".join(i_parts[3:])
    # Using full inertia tensor: diaginertia doesn't take off-diagonal. MuJoCo full inertia attribute is `fullinertia`
    ET.SubElement(body, 'inertial', pos=i_pos, euler=i_euler, mass=l['mass'], fullinertia=l['inertia'])
    
    # Joint (if not base_link)
    if link_name in joints:
        j = joints[link_name]
        j_parts = j['pose'].split()
        j_pos = " ".join(j_parts[:3])
        ET.SubElement(body, 'joint', name=j['name'], type="hinge", pos=j_pos, axis=j['axis'], range=j['range'])
        
    # Geom
    if l['mesh']:
        # If it's a foot, add a collision sphere too
        if "foot" in link_name:
            # We add the mesh as visual only
            ET.SubElement(body, 'geom', type="mesh", mesh=l['mesh'], contype="0", conaffinity="0", group="1")
            # And a sphere for collision (soft contact)
            ET.SubElement(body, 'geom', type="sphere", size="0.02", pos="0 0 -0.05", friction="1 0.005 0.0001", solref="0.02 1", solimp="0.9 0.95 0.001")
        else:
            # Normal mesh collision and visual
            ET.SubElement(body, 'geom', type="mesh", mesh=l['mesh'])
            
    # Recursively add children
    for child_name, j in joints.items():
        if j['parent'] == link_name:
            build_body(body, child_name)

# Find root link (no parent)
children = set(j['child'] for j in joints.values())
root_link = None
for l in links.keys():
    if l not in children:
        root_link = l
        break

if root_link:
    build_body(worldbody, root_link)

# Add actuators
actuator = ET.SubElement(mjcf, 'actuator')
for child_name, j in joints.items():
    name = j['name']
    if "hip" in name:
        kp, kv = "20", "2"
    elif "knee" in name:
        kp, kv = "30", "3"
    else:
        kp, kv = "50", "2"
    ET.SubElement(actuator, 'position', name=name+"_pos", joint=name, kp=kp, kv=kv)

# Write to string
xml_str = ET.tostring(mjcf, encoding='unicode')
# Pretty print (poor man's)
import xml.dom.minidom
dom = xml.dom.minidom.parseString(xml_str)
pretty_xml = dom.toprettyxml(indent="  ")

with open('thex.xml', 'w') as f:
    f.write(pretty_xml)

print("Created thex.xml")
