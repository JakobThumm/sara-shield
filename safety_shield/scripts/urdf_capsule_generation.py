import os
import math
import re
import warnings

import numpy as np
import matplotlib.pyplot as plt
import trimesh
import xml.etree.ElementTree as ET
from sklearn.decomposition import PCA

# make console output more readable
np.set_printoptions(threshold=np.inf)
np.set_printoptions(suppress=True)


def export_PC(point_cloud, name):
    """
    debug function: export point cloud to ply file (can be imported to blender)
    :param point_cloud: the point cloud to export
    :param name: name of the file
    :return: void
    """
    print("saving to file: " + name)
    with open(name + '.ply', "w") as file:
        # HEADER
        file.write("ply\n")
        file.write("format ascii 1.0\n")
        file.write("element vertex {}\n".format(point_cloud.shape[0]))
        file.write("property float x\n")
        file.write("property float y\n")
        file.write("property float z\n")
        file.write("end_header\n")

        # write points
        for p in point_cloud:
            file.write(
                "{} {} {} \n".format(p[0], p[1], p[2]))


def visualize_meshes(*meshes):
    """
    visualize tri meshes for debugging purposes
    :param meshes: all meshes that should be visualized in same plot
    :return: void
    """
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    for mesh in meshes:
        ax.plot_trisurf(mesh.vertices[:, 0], mesh.vertices[:, 1], mesh.vertices[:, 2], triangles=mesh.faces)


def rpy_to_rot_matrix(rpy):
    """
    construct rotation matrix from RPY parameters
    :param rpy: np array with roll, pitch and yaw
    :return: 3x3 array representing the rotation matrix
    """
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])

    Ry = np.array([[np.cos(pitch), 0, -np.sin(pitch)],  # correct?
                   [0, 1, 0],
                   [np.sin(pitch), 0, np.cos(pitch)]])  # correct?

    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])

    R = Rz @ Ry @ Rx
    return R


def get_mesh_filename(link):
    visual = link.find('visual')

    if visual is not None:
        geometry = visual.find('geometry')

        if geometry is not None:
            mesh = geometry.find('mesh')

            if mesh is not None:
                filepath = mesh.attrib.get('filename', '')
                filename = os.path.basename(filepath)

                if 'scale' in mesh.attrib:
                    scale_xyz = [float(x) for x in mesh.attrib.get('scale').split()]
                else:
                    scale_xyz = [1.0, 1.0, 1.0]

                return filename, scale_xyz
    return None, None


def find_trafo_and_mesh_filenames(urdf_file, relevant_joint_type, joint_names_of_interest):
    """
    Find the mesh filenames of parent and child for all relevant FIXED joint.
    Save them into list of tuple (parent, parent_filename, child, child_filename)
    and then return this list as well the transformations between the parent and child.
    :param urdf_file: URDF Filename
    :param joint_names_of_interest: relevant joints. Skip joints for wheels, cameras, ...
    :return: fixed_joints_and_mesh_filenames and transformations
    """
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    fixed_joints_and_mesh_filenames = []
    transformations = {}

    for joint in root.iter('joint'):
        joint_type = joint.attrib.get('type', '')

        if joint_type == relevant_joint_type:
            parent = joint.find('parent').attrib['link']
            child = joint.find('child').attrib['link']

            if not re.search(joint_names_of_interest, child):
                continue
            parent_filename = None
            child_filename = None
            parent_scale = None
            child_scale = None

            # for every joint, we want to find the parent and child link
            # when found the link, we save the filenames of the meshes
            for link in root.iter('link'):
                link_name = link.attrib.get('name', '')

                if link_name == parent:
                    parent_filename, parent_scale = get_mesh_filename(link)

                if link_name == child:
                    child_filename, child_scale = get_mesh_filename(link)

            # save the link names and filenames in fixed_joints_and_mesh_filenames
            fixed_joints_and_mesh_filenames.append(
                (parent, parent_filename, parent_scale, child, child_filename, child_scale))

            # get the translation and rotation between the child and parent frame
            origin = joint.find("origin")
            rpy = np.array([float(x) for x in origin.get("rpy").split()])
            xyz = np.array([float(x) for x in origin.get("xyz").split()])

            rotation_matrix = rpy_to_rot_matrix(rpy)

            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = rotation_matrix
            transformation_matrix[:3, 3] = xyz

            transformations[child] = transformation_matrix  # Use child name as key

    return fixed_joints_and_mesh_filenames, transformations


def generate_fixed_joint_point_clouds(fixed_joints_and_mesh_filenames, transformations, stl_folder):
    """
    Load stl files of parent and child meshes. Transform child mesh and combine point clouds.
    Then return the point clouds.
    :param fixed_joints_and_mesh_filenames: names of fixed parents and children of fixed joints,
                                            as well as their mesh filenames and scales
    :param transformations: transformation between parent and child of fixed joints. Dict with child as key
    :param stl_folder: folder name that contains the stl meshes
    :return: list of point clouds. One numpy array for each joint
    """
    point_clouds = []

    # Load meshes
    parent_meshes = {}
    child_meshes = {}

    for parent, parent_filename, parent_scale, child, child_filename, child_scale in fixed_joints_and_mesh_filenames:
        if parent_filename is not None and parent_filename not in parent_meshes:
            parent_mesh_file = os.path.join(stl_folder, parent_filename)
            if os.path.exists(parent_mesh_file):
                print(parent_mesh_file)
                parent_meshes[parent_filename] = trimesh.load_mesh(parent_mesh_file)
                parent_meshes[parent_filename].apply_scale(parent_scale)

        if child_filename is not None and child_filename not in child_meshes:
            child_mesh_file = os.path.join(stl_folder, child_filename)
            if os.path.exists(child_mesh_file):
                child_meshes[child_filename] = trimesh.load_mesh(child_mesh_file)
                child_meshes[child_filename].apply_scale(child_scale)

    # Generate point clouds
    for parent, parent_filename, _, child, child_filename, _ in fixed_joints_and_mesh_filenames:
        if parent_filename is None or child_filename is None:
            warnings.warn("No mesh file for link" + child + " or " + parent)
            continue

        parent_mesh = parent_meshes.get(parent_filename)
        child_mesh = child_meshes.get(child_filename)

        if parent_mesh is None or child_mesh is None:
            warnings.warn("Mesh could not be loaded for: " + child + " or " + parent)
            continue

        if parent_mesh.vertices.shape[0] > 0 and child_mesh.vertices.shape[0] > 0:

            if child in transformations:
                try:
                    child_transform = transformations[child]

                    transformed_child_mesh = child_mesh.copy()
                    transformed_child_mesh.apply_transform(child_transform)

                    parent_points = parent_mesh.vertices
                    child_points = transformed_child_mesh.vertices

                    point_cloud = np.concatenate([parent_points, child_points])
                    point_clouds.append(point_cloud)

                    # DEBUG: visualize both meshes (parent and transformed child mesh)
                    # visualize_meshes(parent_mesh, transformed_child_mesh)
                    # export_PC(point_cloud=point_cloud, name=parent + "_" + child)

                except Exception as e:
                    print(f"Error processing fixed joint '{parent} -> {child}': {str(e)}")

    plt.show()
    return point_clouds


def generate_joint_point_clouds(fixed_joints_and_mesh_filenames, stl_folder):
    """
    generate points clouds for normal robots (no fixed joints in chain)
    :param fixed_joints_and_mesh_filenames: parent and child of each joint. Only child mesh is used
    :param stl_folder: folder name that contains the stl meshes
    :return: list of point clouds. One numpy array for each joint
    """
    point_clouds = []

    # Load meshes
    child_meshes = {}

    for _, _, _, child, child_filename, child_scale in fixed_joints_and_mesh_filenames:
        if child_filename is not None and child_filename not in child_meshes:
            child_mesh_file = os.path.join(stl_folder, child_filename)
            if os.path.exists(child_mesh_file):
                child_meshes[child_filename] = trimesh.load(child_mesh_file, force='mesh')
                child_meshes[child_filename].apply_scale(child_scale)

    # Generate point clouds
    for _, _, _, child, child_filename, _ in fixed_joints_and_mesh_filenames:
        if child_filename is None:
            warnings.warn("No mesh file for link" + child)
            continue

        child_mesh = child_meshes.get(child_filename)

        if child_mesh is None:
            warnings.warn("Mesh could not be loaded for: " + child)
            continue

        if child_mesh.vertices.shape[0] > 0:
            child_points = child_mesh.vertices

            point_clouds.append(child_points)

    return point_clouds


def capsule_from_points(points: np.ndarray):
    """
    Computes a capsule that encloses a set of points in 3D space.
    :param points: points: An (N, 3) numpy array representing N points in 3D space.
    :return:  A tuple containing the two endpoints of the capsule and the radius.
    """
    # Compute PCA of the points
    pca = PCA(n_components=3)
    pca.fit(points)
    coeff = pca.components_.T
    center = (np.amax(points, axis=0) + np.amin(points, axis=0)) / 2
    direction = -1 * coeff[:, 0]
    center_repmat = np.tile(center, (len(points), 1))
    distances = np.sqrt(np.sum(np.square(
        (center_repmat - points) - np.multiply(np.tile(np.dot((center_repmat - points), direction), (3, 1)).T,
                                               np.tile(direction, (len(points), 1)))), axis=1))
    distance_max = np.max(distances)

    # Compute max length of the capsule
    Z = pca.transform(points)
    max_length = np.amax(Z, axis=0) - np.amin(Z, axis=0)

    # Initialize variables
    p1_min = center
    p2_min = center
    r_min = np.max(np.sqrt(np.sum((np.square(center_repmat - points)), axis=1)))
    volume_min = 4 / 3 * math.pi * (r_min ** 3)

    # Compute optimal capsule by iterating over possible endpoints
    for i in np.arange(0.005, max_length[0] / 2, 0.0005):
        p2 = center + (i * direction)
        p1 = center - (i * direction)
        points_2 = points[(np.dot((points - np.tile(p2, (len(points), 1))), direction) >= 0), :]
        points_1 = points[(np.dot((points - np.tile(p1, (len(points), 1))), direction) < 0), :]
        if len(points_1) == 0 or len(points_2) == 0:
            continue
        r_i1 = np.max(np.sqrt(np.sum((np.square(points_1 - np.tile(p1, (len(points_1), 1)))), axis=1)))
        r_i2 = np.max(np.sqrt(np.sum((np.square(points_2 - np.tile(p2, (len(points_2), 1)))), axis=1)))
        r_i = max([r_i1, r_i2, distance_max])
        volume = 4 / 3 * math.pi * (r_i ** 3) + (math.pi * (r_i ** 2) * np.sqrt(np.dot((p2 - p1), (p2 - p1))))
        if volume < volume_min:
            p1_min = p1
            p2_min = p2
            r_min = r_i
            volume_min = volume

    return p1_min, p2_min, r_min


def parse_urdf_file_transformations(urdf_file, joint_names_of_interest, alternating_fixed_and_revolute_joints):
    """
    Get joint transformations of relevant joints from the urdf file
    and join the fixed joints with the rotating joints
    :param urdf_file: the file name
    :param joint_names_of_interest: relevant joints
    :param alternating_fixed_and_revolute_joints: whether concert like robots are used
    :return: transformation dict
    """
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    transformations = {}
    fixed_transformations = []

    limit_q_min = []
    limit_q_max = []
    limit_v = []

    for joint in root.findall(".//joint"):
        name = joint.attrib['name']
        type = joint.attrib['type']

        # skip irrelevant joints
        if not re.search(joint_names_of_interest, name):
            continue

        origin = joint.find('origin')
        if origin is None:
            continue

        if 'rpy' in origin.attrib:
            rpy = [float(x) for x in origin.attrib['rpy'].split()]
        else:
            rpy = [0, 0, 0]
        if 'xyz' in origin.attrib:
            xyz = [float(x) for x in origin.attrib['xyz'].split()]
        else:
            xyz = [0, 0, 0]

        R = rpy_to_rot_matrix(rpy)

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz

        if type == 'fixed':
            fixed_transformations.append(T)
        else:
            limit = joint.find('limit')
            if limit is not None:
                try:
                    limit_q_min.append(float(limit.attrib['lower']))
                    limit_q_max.append(float(limit.attrib['upper']))
                    limit_v.append(float(limit.attrib['velocity']))
                except KeyError:
                    pass

            transformations[name] = T

    # join transformations of fixed and rotating joins together (mostly for CONCERT robots)
    if alternating_fixed_and_revolute_joints:
        for (t_name, trans), fixed_trans in zip(transformations.items(), fixed_transformations):
            t1 = fixed_trans @ trans
            transformations[t_name] = t1

    return transformations, limit_q_min, limit_q_max, limit_v


def export_capsules(robot_name, secure_radius, nb_joints, capsules, transformations):
    """
    saves robot parameters to yaml file
    :param robot_name: name of robot (used for the filename)
    :param secure_radius:
    :param nb_joints: number of joints
    :param capsules: robot capsules, list of (point1, point2, r)
    :param transformations: dict of {key: string , value: 4x4 array}
    :return: void
    """
    if nb_joints != len(transformations.items()):
        warnings.warn("Warning...........The number of joints does not match the number of transformations!")

    if nb_joints == len(capsules) - 1:  # we ignore the first capsule, since this one is the base
        ignore_first = True
    elif nb_joints == len(capsules):  # normal robot
        ignore_first = False
    else:
        warnings.warn("Warning...........The number of number does not match the number of capsules!")
        ignore_first = False

    filepath = f'robot_parameters_{robot_name}.yaml'
    with open(filepath, 'w') as f:
        f.write(f"robot_name: {robot_name}\n\n")
        f.write(f"nb_joints: {nb_joints}\n\n")
        f.write(f"secure_radius: {secure_radius}\n\n")
        f.write('transformation_matrices: [\n')
        for name, T in transformations.items():
            f.write('#' + name + '\n')
            f.write('\t{:.4f}, {:.4f}, {:.4f}, {:.4f}, \n'.format(T[0, 0], T[0, 1], T[0, 2], T[0, 3]))
            f.write('\t{:.4f}, {:.4f}, {:.4f}, {:.4f}, \n'.format(T[1, 0], T[1, 1], T[1, 2], T[1, 3]))
            f.write('\t{:.4f}, {:.4f}, {:.4f}, {:.4f}, \n'.format(T[2, 0], T[2, 1], T[2, 2], T[2, 3]))
            f.write('\t{:.4f}, {:.4f}, {:.4f}, {:.4f}, \n'.format(T[3, 0], T[3, 1], T[3, 2], T[3, 3]))
            f.write('\t### \n')
        f.write(']\n')
        f.write("enclosures: [\n")
        for i, (p1, p2, r) in enumerate(capsules):
            if i == 0 and ignore_first:
                continue
            f.write(f"{p1[0]:.8f}, {p1[1]:.8f}, {p1[2]:.8f},\n")
            f.write(f"{p2[0]:.8f}, {p2[1]:.8f}, {p2[2]:.8f},\n")
            f.write(f"{r:.8f}")
            if i < len(capsules) - 1:
                f.write(",\n###\n")
            else:
                f.write("\n")
        f.write("]")


def create_robot_file(limit_q_min, limit_q_max, limit_v, nb_joints, robot_name):
    """
    create the trajectory parameters yaml file with joint limits
    :param limit_q_min: the lowest joint position
    :param limit_q_max: the highest joint position
    :param limit_v: the velocity limit
    :param nb_joints: the specified number of joints for sanity checks
    :param robot_name: name of the robot
    :return: void
    """
    if len(limit_v) != nb_joints:
        warnings.warn("Not all joints have velocity limits specified, using default values")
        limit_v = [2] * nb_joints

    if len(limit_q_min) != nb_joints or len(limit_q_max) != nb_joints:
        warnings.warn("Not all joints have min and max positions specified, using default values")
        limit_q_min = [-3] * nb_joints
        limit_q_max = [3] * nb_joints

    a_max_allowed = [10] * nb_joints
    j_max_allowed = [400] * nb_joints
    a_max_ltt = [2] * nb_joints
    j_max_ltt = [15] * nb_joints
    filepath = f'trajectory_parameters_{robot_name}.yaml'
    with open(filepath, 'w') as f:
        f.write(f'robot_name: "{robot_name}"\n\n')
        f.write('# Failsafe information\n')
        f.write(f'max_s_stop: 0.2\n')
        f.write(f'q_min_allowed: {limit_q_min}\n')
        f.write(f'q_max_allowed: {limit_q_max}\n')
        f.write(f'v_max_allowed: {limit_v}\n')
        f.write(f'a_max_allowed: {a_max_allowed}\n')
        f.write(f'j_max_allowed: {j_max_allowed}\n')
        f.write('# Maximum long-term planning acc / jerk\n')
        f.write(f'a_max_ltt: {a_max_ltt}\n')
        f.write(f'j_max_ltt: {j_max_ltt}\n\n')
        f.write(f'nb_joints: {nb_joints}\n\n')
        for i in range(1, nb_joints + 1):
            f.write(f'q{i}: [0.0]\n')


if __name__ == '__main__':
    '''
    CONCERT Chain:
    mobile_base - J1_E_stator - L1_E - J2_E_stator - L2_E - .....   J99_E_stator - L_99_E      -          end_effector_E
        |  fixed_J1_E  |   J1_E  |  fixed_J2_E |             ....          |  J99_E  |  end_effector_E_fixed_joint  | 
        
    In total   (fixed - rotating) (5 or 6 times) and two fixed joint at the end    
    The last fixed join we skip, since ee_E has no physical extend.
    '''

    # Things to edit
    urdf_file = "panda_arm.urdf"
    stl_folder = "meshes/visual"
    robot_name = "panda"
    alternating_fixed_and_revolute_joints = False  # set to True for CONCERT

    # All relevant joints contain this regex, while other joints don't (e.g. "J[0-9]_E")
    # joint_names_of_interest = "_E"   # use this for CONCERT
    joint_names_of_interest = ""   # use this when all joints should be used

    number_joints = 7
    secure_radius = 0.02

    # for fixed joints: combine two point-clouds: find the mesh-filenames, and the transformations
    # between them and then join them.
    if alternating_fixed_and_revolute_joints:
        mesh_filenames, transformations = find_trafo_and_mesh_filenames(urdf_file, 'fixed', joint_names_of_interest)
        pc = generate_fixed_joint_point_clouds(mesh_filenames, transformations, stl_folder)
    else:
        # for other joints, use only the child mesh of each revolute join
        mesh_filenames, transformations = find_trafo_and_mesh_filenames(urdf_file, 'revolute', joint_names_of_interest)
        pc = generate_joint_point_clouds(mesh_filenames, stl_folder)

    # generate capsules for every point cloud
    capsules = []
    for points in pc:
        p1, p2, r = capsule_from_points(points)
        capsules.append((p1, p2, r))

    # get the transformations between each link (fixed + rotating)
    # also get the min and max position of joints, as well as the max velocity
    trafo, limit_q_min, limit_q_max, limit_v = (
        parse_urdf_file_transformations(urdf_file, joint_names_of_interest, alternating_fixed_and_revolute_joints))

    for name, T in trafo.items():
        print(f"{name} transformation matrix:")
        print(np.round(T, decimals=4))

    # generate output
    export_capsules(robot_name, secure_radius, number_joints, capsules, trafo)
    create_robot_file(limit_q_min, limit_q_max, limit_v, number_joints, robot_name)
