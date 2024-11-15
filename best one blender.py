import bpy
import bmesh
import os
import math
from mathutils import Vector, Euler
import numpy as np

def create_slices_from_orientation(obj, num_slices):
    euler_angles = obj.rotation_euler
    normal_vector = euler_angles.to_matrix() @ Vector((0, 0, 1))
    normal_vector.normalize()

    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True) 

    bbox = [obj.matrix_world @ Vector(corner) for corner in obj.bound_box]

    min_coord = min(bbox, key=lambda v: v.dot(normal_vector))
    max_coord = max(bbox, key=lambda v: v.dot(normal_vector))

    min_dot = min_coord.dot(normal_vector)
    max_dot = max_coord.dot(normal_vector)

    slice_spacing = (max_dot - min_dot) / num_slices
    
    sliced_objects = []
    
    num = 3
    
    for i in range(num, num_slices):
        dot_loc = min_dot + i * slice_spacing
        point_on_plane = min_coord + normal_vector * (i * slice_spacing)

        bpy.ops.mesh.primitive_plane_add(size=9, location=point_on_plane)
        plane = bpy.context.object
        quat = normal_vector.to_track_quat('Z', 'Y')
        plane.rotation_euler = quat.to_euler()

        obj_copy = obj.copy()
        obj_copy.data = obj.data.copy()
        bpy.context.collection.objects.link(obj_copy)
        sliced_objects.append(obj_copy)
        mod = obj_copy.modifiers.new(name=f'Slice_{i}', type='BOOLEAN')
        mod.operation = 'INTERSECT'
        mod.object = plane
        bpy.context.view_layer.objects.active = obj_copy
        bpy.ops.object.modifier_apply(modifier=mod.name)

        bpy.data.objects.remove(plane, do_unlink=True)
    
    return sliced_objects

def extract_vertices_and_normals(obj):
    vertices_and_normals = [] 
    for vertex in obj.data.vertices:
        world_vertex = obj.matrix_world @ vertex.co
        world_normal = obj.matrix_world.to_3x3() @ vertex.normal
        world_normal.normalize()
        
        vertices_and_normals.append((world_vertex.x, world_vertex.y, world_vertex.z, world_normal.x, world_normal.y, world_normal.z))
    
    return vertices_and_normals

def generate_gcode(vertices_and_normals):
    gcode = []
    for v in vertices_and_normals:
        x, y, z, i, j, k = v
        gcode.append(f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} I{i:.3f} J{j:.3f} K{k:.3f}")
    return gcode

def five_axis_transform(g, offset=0.068):
    if len(g) != 7 or g[0] not in ['G0', 'G1']:
        raise ValueError("Invalid G-code command")
    norm = np.sqrt(g[4]**2 + g[5]**2 + g[6]**2)
    
    if norm == 0:
        return printer(('G1', 0, 0, 0, 0, 0))
    
    unit_vec = [g[4] / norm, g[5] / norm, g[6] / norm]
    
    b = np.arccos(unit_vec[2])
    c = np.arctan2(unit_vec[1], unit_vec[0])
    
    Tran_b = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, -offset],
        [0, 0, 0, 1]
    ])
    RotB = np.array([
        [np.cos(b), 0, np.sin(b), 0],
        [0, 1, 0, 0],
        [-np.sin(b), 0, np.cos(b), 0],
        [0, 0, 0, 1]
    ])
    RotC = np.array([
        [np.cos(c), -np.sin(c), 0, 0],
        [np.sin(c), np.cos(c), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    Tran_a = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, -offset],
        [0, 0, 0, 1]
    ])
    
    Rot = RotB @ RotC
    T = Tran_b @ Rot @ Tran_a
    
    coord_mat = np.array([[g[1]], [g[2]], [g[3]], [1]])
    updated_coord = T @ coord_mat
    
    output_mat = [[updated_coord[0][0]], [updated_coord[1][0]], [updated_coord[2][0]]]
    
    print(f"Original Coordinates: {g[1:4]}")
    print(f"Transformed Coordinates: {output_mat}")
    print(f"B: {np.degrees(b)}, C: {np.degrees(c)}")
                  
    return printer(('G1', 'F800', output_mat[0][0], output_mat[1][0], output_mat[2][0], np.degrees(b), np.degrees(c), 'A0'))

def printer(data, offset=0.068):
    final_transformation = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, offset],
        [0, 0, 0, 1]
    ])
    final_coord = np.array([[data[2]], [data[3]], [data[4]], [1]])
    result_coord = final_transformation @ final_coord
    
    result_mat = [[result_coord[0][0]], [result_coord[1][0]], [result_coord[2][0]]]
                  
    result = f"{data[0]} {data[1]} X{result_mat[0][0]:.4f} Y{result_mat[1][0]:.4f} Z{result_mat[2][0]:.4f} B{data[5]:.4f} C{data[6]:.4f} {data[7]}"
    return result

def breaker(s):
    parts = s.split()
    result = (parts[0],)
    for part in parts[1:]:
        result += (float(part[1:]),)
    return result

def main():
    if bpy.context.object.mode != 'OBJECT':
        bpy.ops.object.mode_set(mode='OBJECT')

    obj = bpy.context.active_object
    if not obj:
        print("No active object selected")
        return
    
    num_slices = 20
      
    sliced_objects = create_slices_from_orientation(obj, num_slices)

    all_gcode = []
    for i, sliced_obj in enumerate(sliced_objects):
        vertices_and_normals = extract_vertices_and_normals(sliced_obj)
        gcode = generate_gcode(vertices_and_normals)
        all_gcode.extend(gcode)

        print(f"G-code for Sliced Object {i+1}:\n")
        print("\n".join(gcode))
        print("\n")

    initialization_gcode = ["G28 ;Home\nG92 E0\nM107"]    
    
    five_axis_gcode = []
    for i in all_gcode:
        five_axis_gcode.append(five_axis_transform(breaker(i)))
    
    final_commands = [
        "M104 S0",
        "M140 S0",
        "G92 E1",
        "G1 E-1 F1500",
        "G28 X0 Y0",
        "M84"
    ]

    write_directory = os.path.expanduser("~/Documents")
    output_file_path = os.path.join(write_directory, "output_gcode.txt")

    with open(output_file_path, "w") as f:
        f.write("\n".join(initialization_gcode) + "\n")
        f.write("\n".join(five_axis_gcode) + "\n")
        f.write("\n".join(final_commands) + "\n")

    print(f"G-code file saved at: {output_file_path}")

if __name__ == "__main__":
    main()