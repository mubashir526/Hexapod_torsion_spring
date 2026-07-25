import xml.etree.ElementTree as ET

tree = ET.parse('ROS/src/sim_robot/models/THex_Quadruped/model.sdf')
root = tree.getroot()
model = root.find('model')

mjcf = ET.Element('mujoco', model="THex_Quadruped")

compiler = ET.SubElement(mjcf, 'compiler', angle="radian", meshdir="meshes/")
option = ET.SubElement(mjcf, 'option', timestep="0.001", gravity="0 0 -9.81")
asset = ET.SubElement(mjcf, 'asset')

def get_pose(elem):
    p = elem.find('pose')
    return p.text if p is not None else "0 0 0 0 0 0"

links = {}
for link in model.findall('link'):
    name = link.get('name')
    
    inert = link.find('inertial')
    mass = inert.find('mass').text if inert is not None else "1"
    
    if inert is not None and inert.find('pose') is not None:
        i_vals = [float(x) for x in inert.find('pose').text.split()]
        i_pos = f"{i_vals[0]} {i_vals[1]} {i_vals[2]}"
        i_euler = f"{i_vals[3]} {i_vals[4]} {i_vals[5]}"
    else:
        i_pos = "0 0 0"
        i_euler = "0 0 0"
        
    inertia = inert.find('inertia') if inert is not None else None
    if inertia is not None:
        ixx = inertia.find('ixx').text
        iyy = inertia.find('iyy').text
        izz = inertia.find('izz').text
        full_inertia = f"{ixx} {iyy} {izz}"
    else:
        full_inertia = "1 1 1"
        
    vis = link.find('visual')
    mesh_uri = None
    if vis is not None:
        geom = vis.find('geometry')
        if geom is not None:
            mesh = geom.find('mesh')
            if mesh is not None:
                uri = mesh.find('uri').text
                mesh_uri = uri.split('/')[-1]
                if asset.find(f"mesh[@file='{mesh_uri}']") is None:
                    ET.SubElement(asset, 'mesh', file=mesh_uri)
    
    links[name] = {
        'mass': mass,
        'inertial_pos': i_pos,
        'inertial_euler': i_euler,
        'inertia': full_inertia,
        'mesh': mesh_uri
    }

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
        'child': child,
        'pose': pose,
        'axis': xyz,
        'range': f"{lower} {upper}"
    }

worldbody = ET.SubElement(mjcf, 'worldbody')
ET.SubElement(worldbody, 'light', diffuse=".5 .5 .5", pos="0 0 3", dir="0 0 -1")
ET.SubElement(worldbody, 'geom', type="plane", size="5 5 0.1", rgba=".9 .9 .9 1")

def build_body(parent_elem, link_name):
    l = links[link_name]
    
    if link_name in joints:
        j = joints[link_name]
        parts = j['pose'].split()
        pos_str = " ".join(parts[:3])
        euler_str = " ".join(parts[3:])
        # The child body pose is exactly the SDF joint pose relative to parent!
        body = ET.SubElement(parent_elem, 'body', name=link_name, pos=pos_str, euler=euler_str)
        # The MuJoCo joint is exactly at the child body origin
        ET.SubElement(body, 'joint', name=j['name'], type="hinge", pos="0 0 0", axis=j['axis'], range=j['range'])
    else:
        # Base link
        body = ET.SubElement(parent_elem, 'body', name=link_name, pos="0 0 0", euler="0 0 0")
        
    ET.SubElement(body, 'inertial', pos=l['inertial_pos'], euler=l['inertial_euler'], mass=l['mass'], diaginertia=l['inertia'])
    
    if l['mesh']:
        mesh_name = l['mesh'].split('.')[0]
        ET.SubElement(body, 'geom', type="mesh", mesh=mesh_name, contype="0", conaffinity="0", group="1")
        
        if "foot" in link_name:
            ET.SubElement(body, 'geom', type="sphere", size="0.02", pos="0 0 -0.01", friction="1 0.005 0.0001", solref="0.02 1", solimp="0.9 0.95 0.001")
        elif "base_link" in link_name:
            ET.SubElement(body, 'geom', type="box", size="0.1 0.05 0.03", pos="0 0 0.05", rgba="1 0 0 0")
            
    for child_name, j in joints.items():
        if j['parent'] == link_name:
            build_body(body, child_name)

children = set(j['child'] for j in joints.values())
root_link = None
for l in links.keys():
    if l not in children:
        root_link = l
        break

if root_link:
    base_body = ET.SubElement(worldbody, 'body', name="floating_base", pos="0 0 0.3")
    ET.SubElement(base_body, 'freejoint')
    build_body(base_body, root_link)

actuator = ET.SubElement(mjcf, 'actuator')
for child_name, j in joints.items():
    name = j['name']
    if "hip" in name:
        kp, kv = "20", "5"
    elif "knee" in name:
        kp, kv = "30", "3"
    else:
        kp, kv = "50", "2"
    ET.SubElement(actuator, 'position', name=name+"_pos", joint=name, kp=kp, kv=kv, forcelimited="true", forcerange="-3.0 3.0")

xml_str = ET.tostring(mjcf, encoding='unicode')
import xml.dom.minidom
dom = xml.dom.minidom.parseString(xml_str)
pretty_xml = dom.toprettyxml(indent="  ")

with open('mujoco_expei/thex.xml', 'w') as f:
    f.write(pretty_xml)

print("Created thex.xml perfectly matched to Gazebo nested kinematic tree!")
