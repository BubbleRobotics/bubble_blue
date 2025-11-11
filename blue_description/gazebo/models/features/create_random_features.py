#!/usr/bin/env python3
import random
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

def feature_profile(z, depth=5.05):
    """
    Returns the shape value f(x) for a double-arched feature.
    x is assumed to be between 0 and 1.
    """
    z = z + depth  # shift z to start from 0
    z = z / 1.5  # normalize to [0,1] 
    # parameters
    y0 = 0.11 # minimum height
    ymax = 0.24 # maximum height
    A = ymax - y0 # amplitude
    
    # define intervals
    if z < 0.0 or z > 1.0:
        return y0
    
    # first arch
    if 0.0 <= z <= 0.5:
        t = (z - 0.05) / (0.5 - 0.05)
        return y0 + A * np.sin(np.pi * t)
    
    # second arch
    if 0.5 < z <= 1.0:
        t = (z - 0.5) / (0.95 - 0.5)
        return y0 + A * np.sin(np.pi * t)
    
    return y0

def add_bumps_to_sdf(
    sdf_path: Path,
    link_name: str = "features",
    n: int = 50,
    h_range=(0.05, 0.95),
    n_base=0.15,
    n_jitter=(-0.001, 0.001),
    v_range=(0.05, 0.9),
    size_x=(0.015, 0.05),
    size_y=(0.015, 0.05),
    size_z=(0.015, 0.05),
    roll=(0.0, 3.14),
    pitch=(0.0, 3.14),
    yaw=(0.0, 3.14),
    seed: int | None = None, skip_after: int = 1
):
    if seed is not None:
        random.seed(seed)

    sdf_path = Path(sdf_path)
    if not sdf_path.exists():
        raise FileNotFoundError(f"No such file: {sdf_path}")

    # Backup
    bak_path = sdf_path.with_suffix(sdf_path.suffix + ".bak")
    bak_path.write_bytes(sdf_path.read_bytes())

    # Parse XML
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    # --- Find or create single link ---
    links = [lnk for lnk in root.iter("link") if lnk.get("name") == link_name]
    if len(links) == 0:
        raise ValueError(f"Link '{link_name}' not found in {sdf_path}")
    target_link = links[0]

    # Remove any duplicate <link name="features"> that might exist
    parent = root.find(".//model")
    for lnk in list(parent.findall("link")):
        if lnk is not target_link and lnk.get("name") == link_name:
            parent.remove(lnk)

    # --- Clear old visuals if desired ---
    for old_vis in list(target_link.findall("visual")):
        target_link.remove(old_vis)

    # Define geometry basis
    bot_left = np.array([11.835702, 13.253233, -5])
    next_bot_left = np.array([11.319732, 11.416660, -5])
    bot_right = bot_left + (next_bot_left - bot_left) / 2
    top_left = np.array([11.835702, 13.253233, -3.5])

    horizontal_vec = bot_right - bot_left
    vertical_vec = top_left - bot_left
    normal_vec = np.cross(horizontal_vec, vertical_vec)
    normal_vec_norm = normal_vec / np.linalg.norm(normal_vec)

    bumps = 0
    for j in range(16): # iterate over all 16 inspection modules
        if j <= 7:
            if j >= skip_after: 
                continue  # after 3rd skip to lower row (distances to not match up)
            offset = j * horizontal_vec * 2 # upper row (move right by 2x horizontal vector)
            depdth = 5.05
        else:
            if j >= 8 + skip_after: 
                continue  # after 3rd end
            offset = (j-8) * horizontal_vec * 2
            offset[2] -= 4 # lower row is 4m lower
            depdth = 9.05

        for i in range(n): # bumps per inspection module
            bumps += 1
            xyz = (bot_left + offset + random.uniform(*h_range) * horizontal_vec + random.uniform(*v_range) * vertical_vec) 
            xyz = xyz + (feature_profile(xyz[2],depdth) + random.uniform(*n_jitter)) * normal_vec_norm 
            print(feature_profile(xyz[2],depdth))
            print(f"Bump {i}: position {xyz}") 
            
            sx = random.uniform(*size_x) 
            sy = random.uniform(*size_y) 
            sz = random.uniform(*size_z) 
            r = random.uniform(*roll) 
            p = random.uniform(*pitch) 
            yw = random.uniform(*yaw)

            visual = ET.Element("visual", {"name": f"feature_{j}_bump_{i}"})
            pose = ET.SubElement(visual, "pose")
            pose.text = f"{xyz[0]} {xyz[1]} {xyz[2]} {r} {p} {yw}"

            geom = ET.SubElement(visual, "geometry")
            box = ET.SubElement(geom, "box")
            size = ET.SubElement(box, "size")
            size.text = f"{sx} {sy} {sz}"

            ET.SubElement(visual, "cast_shadows").text = "true"

            material = ET.SubElement(visual, "material")
            ambient = ET.SubElement(material, "ambient")
            ambient.text = "0.8 0.8 0.8 1"
            diffuse = ET.SubElement(material, 'diffuse')
            diffuse.text  = '0.22 0.22 0.22 1'
            specular = ET.SubElement(material, 'specular')
            specular.text = '0.02 0.02 0.02 1'

            pbr = ET.SubElement(material, "pbr")
            metal = ET.SubElement(pbr, "metal")
            rough = ET.SubElement(metal, "roughness")
            metalness = ET.SubElement(metal, "metalness")
            rough.text = "0.8"
            metalness.text = "0.02"

            target_link.append(visual)
        

    # --- Pretty formatting ---
    def indent(elem, level=0):
        i = "\n" + level * "  "
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "  "
            for child in elem:
                indent(child, level + 1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    indent(root)
    tree.write(sdf_path, encoding="utf-8", xml_declaration=True)
    print(f"Wrote {bumps} bumps to {sdf_path} (and backed up to {bak_path})")

if __name__ == "__main__": 
    # Defaults assume the script lives beside model.sdf 
    here = Path(__file__).resolve().parent 
    sdf_file = here / 'model.sdf' 
    add_bumps_to_sdf(sdf_file, link_name='features')