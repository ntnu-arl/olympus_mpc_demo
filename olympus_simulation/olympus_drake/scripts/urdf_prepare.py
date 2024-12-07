import os
import re
import trimesh 

#this script takes the exported urdf with the corresponding stl meshes and 
#creates a new urdf for simulation purposes and creates the obj meshes that are compatible with drale

# REQUIRES NUMPY VER 1.21 to use trimesh. 
# `$ pip install numpy==1.21`

## inputs - absolute path
olympus_src_path   = '/home/papaveneti/thesis_mpc/ros_ws/src/'
olympus_input_urdf = 'olympus_simple_collision.urdf'
mesh_path          = 'meshes'


def convert_stl_to_obj(stl_file, obj_file):
    # Load the STL file
    mesh = trimesh.load(stl_file)
    
    # Export the mesh to OBJ format
    mesh.export(obj_file)
    print(f"Converted {stl_file} to {obj_file}")

def update_mesh_paths_and_convert(urdf_file, output_file, package_name, new_path, stl_base_directory, obj_base_directory):
    # Read the URDF file
    with open(urdf_file, 'r') as file:
        urdf_content = file.read()

    # Define the regex pattern to match the package paths
    pattern = re.compile(r'package://' + re.escape(package_name) + r'/urdf/meshes/([^"]+\.STL)', re.IGNORECASE)

    # Function to replace .STL with .obj and convert files
    def replace_and_convert(match):
        stl_relative_path = match.group(1)
        stl_file_path = os.path.join(stl_base_directory, stl_relative_path)
        
        # Determine the corresponding .obj filename and path
        obj_relative_path = os.path.splitext(stl_relative_path)[0] + '.obj'
        obj_file_path = os.path.join(obj_base_directory, obj_relative_path)
        
        # Ensure the directory for the OBJ file exists
        obj_dir = os.path.dirname(obj_file_path)
        os.makedirs(obj_dir, exist_ok=True)
        
        # Convert the file
        convert_stl_to_obj(stl_file_path, obj_file_path)
        
        # Return the new path for the URDF
        return f'{new_path}/{obj_relative_path}'

    # Replace all occurrences and convert STL to OBJ
    updated_urdf_content = re.sub(pattern, replace_and_convert, urdf_content)

    # Write the updated content to a new URDF file
    with open(output_file, 'w') as file:
        file.write(updated_urdf_content)

    print(f"Updated URDF saved to {output_file}")

#Paths:
output_urdf_pkg_relative_path = 'olympus_sim/olympus_drake/urdf/olympus_drake.urdf'

urdf_file   = olympus_src_path + 'olympus/olympus_description/urdf/' +  olympus_input_urdf
output_file = olympus_src_path + output_urdf_pkg_relative_path
package_name = 'olympus_description'
stl_base_directory = olympus_src_path+'olympus/olympus_description/urdf/meshes'
obj_base_directory = olympus_src_path+'olympus_sim/olympus_drake/urdf/' + mesh_path

update_mesh_paths_and_convert(urdf_file, output_file, package_name, mesh_path, stl_base_directory, obj_base_directory)

