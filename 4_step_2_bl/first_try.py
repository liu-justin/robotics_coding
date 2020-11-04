import mr 
import untangle
import numpy as np

# first untagnle the xml file, then create the inputs for ch4 forward kinematics
obj = untangle.parse("6DoF_URDF.xml")

# initialize TList, bodyList from obj
TList = []
bodyList = np.array([0,0,0,0,0,0])

np.set_printoptions(precision=7, suppress=True)

# grabbing the last joint (ee_joint) R and p, inserting into TList
rpy_ee = [float(n) for n in obj.robot.joint[len(obj.robot.joint)-1].origin["rpy"].split()]
R_ee = mr.RollPitchYawToRot(rpy_ee[0],rpy_ee[1],rpy_ee[2])
p_ee = [float(n) for n in obj.robot.joint[len(obj.robot.joint)-1].origin["xyz"].split()]
T_ee = mr.RpToTrans(R_ee, p_ee)

# skips all joints that are type fixed (like the base link and ee_link)
joint_list = [joint for joint in obj.robot.joint if joint["type"]!="fixed"]

# need to reverse jointlist to get T_joint_ee, to get bodyList properly
for joint in reversed(joint_list):
    # find the roll-pitch-yaw, about the z-y-x axes of the previous joint ; convert to a rotation matrix
    rpy = [float(n) for n in joint.origin["rpy"].split()]
    R = mr.RollPitchYawToRot(rpy[0], rpy[1], rpy[2])

    # find the distance from previous joint to current joint
    p = np.array([float(n) for n in joint.origin["xyz"].split()])

    # this T takes previous joint to current joint, or is current joint relative to prev joint
    # T_56, T_lower_higher
    T = mr.RpToTrans(R,p)
    TList.insert(0,T)

    # T_ee is end_effector joint relative to current joint, need inverse of that to get v
    (R_ee, p_ee) = mr.TransToRp(mr.TransInv(T_ee))

    # find which axis the motor at this joint turns about
    current_omega = [float(n) for n in joint.axis["xyz"].split()]
    # convert the axis into ee_frame
    ee_omega = np.dot(R_ee, current_omega)
    # skew symmetric it
    ee_omega_skewed = mr.VecToso3(ee_omega)

    # negative one here just works somehow
    current_v = -1*np.dot(ee_omega_skewed, p_ee)

    # combine w,v into body_axis, then insert into bodyList
    body_axis = np.r_[current_omega, current_v]
    bodyList = np.c_[body_axis, bodyList]
    print(f"bodyaxis: {body_axis}")

    # update T_ee to be relative to current link T_56 * T_6ee = T_5ee
    T_ee = np.dot(T, T_ee)

# remove the filler column at initialization needed to start appending
bodyList = np.delete(bodyList, len(bodyList[0])-1,1)

##### inverse dynamics #####
# need Glist, or spatial inertai matrix list
#   6x6 matrix, top left corner is 3x3 rotational inertia matrix, bottom right is mass of link * identity matrix
GList = []

# urdf file has world link and ee_link, so that all joints have a parent and child
# also we dont need the base link and link from joint 6 to ee_link called link6 (remember 6 joints should only have 5 links)
# so for this for loop, skip the first two and last two
for link in obj.robot.link[2:-2]: 
    mass = float(link.inertial.mass["value"])

# got these values from solidworks, similar enough to eigenvectors of rotational inertia matrix
    # ix = [float(n) for n in link.inertial.origin["ix"].split()]
    # iy = [float(n) for n in link.inertial.origin["iy"].split()]
    # iz = [float(n) for n in link.inertial.origin["iz"].split()]
    # principle_axes = np.c_[ix,iy,iz]

    # translate from parent joint to CoM 
    # negative one b/c this is from parent link origin to CoM, but I need CoM to parent link origin
    xyz_CoM= -1*np.array([float(n) for n in link.inertial.origin["xyz"].split()])

    # grab Ixx, Ixy, Ixz, Iyy, Iyz, Izz about the CoM, with the parent link coordinate systems
    inertia_values_CoM = [float(n) for n in (vars(link.inertial.inertia_CoM)["_attributes"].values())]
    
    # putting those values into a rotational inertia matrix, centered at CoM, using parent link coords
    I_CoM = np.array([[inertia_values_CoM[0], inertia_values_CoM[1], inertia_values_CoM[2]],
                      [inertia_values_CoM[1], inertia_values_CoM[3], inertia_values_CoM[4]],
                      [inertia_values_CoM[2], inertia_values_CoM[4], inertia_values_CoM[5]]])

    # grabbing the eigenvectors of the rotational inertia matrix, to find the principle axes of inertia
    w,v = np.linalg.eig(I_CoM) 
    # rotational inertia matrix, centered at CoM, aligned w/ principle axes of inertia
    rotated_I_CoM = np.transpose(v) @ I_CoM @ v
    # print(f"eigenvectors:\n {v}")  
    # print(f"inertia: \n{I_CoM}")
    # print(f"inertia about rotated coords: \n{rotated_I_CoM}")

    # rotational inertia matrix, centered at parent link origin, aligned w/ parent link origin coords
    translated_T_CoM = I_CoM + mass*(np.inner(xyz_CoM, xyz_CoM)*np.identity(3) - np.outer(xyz_CoM, xyz_CoM))
    # print(f"inertial rotational matrix at parent link: \n{translated_T_CoM}")

    # translated_T_CoM is pretty close to the value obtained from SOLIDWORKS
    # inertia_values_joint = [float(n) for n in (vars(link.inertial.inertia_joint)["_attributes"].values())]
    # I_joint = np.array([[inertia_values_joint[0], inertia_values_joint[1], inertia_values_joint[2]],
    #                     [inertia_values_joint[1], inertia_values_joint[3], inertia_values_joint[4]],
    #                     [inertia_values_joint[2], inertia_values_joint[4], inertia_values_joint[5]]])
 
    mI = mass*np.identity(3)
    zeros = np.zeros((3,3))
    Gi = np.c_[np.r_[rotated_I_CoM, zeros], np.r_[zeros,mI]]
    GList.append(Gi)

###### use a joint trajectory to get from rest to home #####
# initialize the theta arrays
theta_rest = np.array([0,0,0,0,0,0])
theta_home = np.array([0,-1*np.pi/2, np.pi/2, 0,0,0])

# parameters for joint trajectory
T_final = 3
N = 50
method = 5

# create the trajectory, N x n matrix where each row is  n-vector of joint variables at an instant in time
trajectory = mr.JointTrajectory(theta_rest, theta_home, T_final, N, method)

###### use a cartesian trajectory to get other places #####

# finding the home config in SE(3)
M_home = mr.FKinBody(T_ee, bodyList, theta_home)

# picking a new config in SE(3), this can be anything
M_new = np.array([[1,0,0,0.5],
                  [0,1,0,0.1],
                  [0,0,1,0],
                  [0,0,0,1]])

trajectory_SE3 = mr.CartesianTrajectory(M_home, M_new, T_final, N, method)
print(trajectory_SE3)

angleList = [theta_home]
for traj in trajectory_SE3:
    current_solution, feasible = mr.IKinBody(bodyList, T_ee, traj, angleList[-1], 0.01, 0.001)
    if feasible:
        angleList.append(current_solution)

print(angleList)
