#!/usr/bin/env python3

import random

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import uuid
import argparse


def main():

    # -------------------------------
    # Initialization
    # -------------------------------
    parser = argparse.ArgumentParser(description='Script to compute perfect numbers.')
    parser.add_argument('-l', '--location', type=str, help='', required=False,
                        default='on_bed')
    parser.add_argument('-o', '--object', type=str, help='', required=False,
                        default='person_standing')

    args = vars(parser.parse_args())  # creates a dictionary
    print(args)

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('robutler_description_psr') + '/models/'

    # Defines poses where to put objects
    poses = {}

    #BEDROOM POSES

    # on bed pose
    p = Pose()
    p.position = Point(x=-6.033466, y=1.971232, z=0.644345)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed'] = {'pose': p}

    # on bed-side-table pose
    p = Pose()
    p.position = Point(x=-4.489786, y=2.867268, z=0.679033)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed_side_table'] = {'pose': p}

    #on desk pose
    p = Pose()
    p.position = Point(x=-8.995759, y=1.971230, z=0.739265)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_desk'] = {'pose': p}

    #on bedroom_chair pose
    p = Pose()
    p.position = Point(x=-8.222350, y=-4.442423, z=0.358929)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bedroom_chair'] = {'pose': p}

    #beside_bed pose
    p = Pose()
    p.position = Point(x=-7.644774, y=2.008019, z=0)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['beside_bed'] = {'pose': p}

    #beside_ball pose
    p = Pose()
    p.position = Point(x=-5.850516, y=-3.788024, z=0)
    q = quaternion_from_euler(0, 0, -2.562000)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['beside_ball'] = {'pose': p}


    #KITCHEN POSES

    # on kitchen counter
    p = Pose()
    p.position = Point(x=8.905399, y=-4.215331, z=0.899402)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_kitchen_counter'] = {'pose': p}

    # on kitchen table
    p = Pose()
    p.position = Point(x=6.068495, y=0.923506, z=0.796716)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_kitchen_table'] = {'pose': p}

    # beside counter pose
    p = Pose()
    p.position = Point(x=7.765609, y=-4.401236, z=0)
    q = quaternion_from_euler(0, 0, -0.015656)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['beside_counter'] = {'pose': p}

    # beside table pose
    p = Pose()
    p.position = Point(x=7.813824, y=1.062184, z=0)
    q = quaternion_from_euler(0, 0, -1.551079)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['beside_table'] = {'pose': p}

    # in front of fridge pose
    p = Pose()
    p.position = Point(x=7.849320, y=-0.806737, z=0)
    q = quaternion_from_euler(0, 0, 1.521186)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['in_front_of_fridge'] = {'pose': p}


    #LIVING ROOM POSES

    #on sofa
    p = Pose()
    p.position = Point(x=-0.173514, y=-1.075620, z=0.482044)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_sofa'] = {'pose': p}

    #on side table
    p = Pose()
    p.position = Point(x=1.101953, y=-1.597206, z=0.359202)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_side_table'] = {'pose': p}

    #on shelf table
    p = Pose()
    p.position = Point(x=4.201159, y=-5.196661, z=0.362344)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_shelf'] = {'pose': p}

    #lying on the couch pose
    p = Pose()
    p.position = Point(x=-0.471327, y=-2.465163, z=0.705599)
    q = quaternion_from_euler(-1.592561, -0.028150, -1.597580)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['lying_on_couch'] = {'pose': p}


    #GYM POSES

    #working_out
    p = Pose()
    p.position = Point(x=4.370349, y=3.157866, z=0.362335)
    q = quaternion_from_euler(-1.465112, 0.014850, 1.579997)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['working_out'] = {'pose': p}

    #under weight bench pose
    p = Pose()
    p.position = Point(x=3.407065, y=2.989202, z=0.045059)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['under_the_weight_bench'] = {'pose': p}





    # define objects
    objects = {}

    # add object sphere_v
    f = open(package_path + 'sphere_v/model.sdf', 'r')
    objects['sphere_v'] = {'name': 'sphere_v', 'sdf': f.read()}

    # add object person_standing
    f = open(package_path + 'person_standing/model.sdf', 'r')
    objects['person_standing'] = {'name': 'person_standing', 'sdf': f.read()}

    # add object blue_cube
    f = open(package_path + 'blue_cube/model.sdf', 'r')
    objects['blue_cube'] = {'name': 'blue_cube', 'sdf': f.read()}


    # Check if given object and location are valid

    if not args['location'] in poses.keys():
        print('Location ' + args['location'] +
              ' is unknown. Available locations are ' + str(list(poses.keys())))

    if not args['object'] in objects.keys():
        print('Object ' + args['object'] +
              ' is unknown. Available objects are ' + str(list(objects.keys())))

    # -------------------------------
    # ROS
    # -------------------------------

    rospy.init_node('insert_object', log_level=rospy.INFO)

    service_name = 'gazebo/spawn_sdf_model'
    print('waiting for service ' + service_name + ' ... ', end='')
    rospy.wait_for_service(service_name)
    print('Found')

    service_client = rospy.ServiceProxy(service_name, SpawnModel)

    print('Spawning an object ...')
    uuid_str = str(uuid.uuid4())
    service_client(objects[args['object']]['name'] + '_' + uuid_str,
                   objects[args['object']]['sdf'],
                   objects[args['object']]['name'] + '_' + uuid_str,
                   poses[args['location']]['pose'],
                   'world')

    print('Done')


if __name__ == '__main__':
    main()
