#!/usr/bin/env python3

import rospy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult
from functools import partial
import subprocess
from std_msgs.msg import String

server = None
marker_pos = 0

menu_handler = MenuHandler()

pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

rooms = ['kitchen','living_room','bedroom','dining_room','gym'] 

locations = ["on_bed", "on_bed_side_table"]

objects = ['sphere_v', 'book_pink', 'beer', 'parrot_bebop_2', 'hammer','lamp_table_small','mug_beer','newspaper_3','can_fanta','donut_1','person_standing']# from spawn_object.py

missions = [
            "find a purple sphere",
            "check if the pc is in the office",
            "check if there is someone in the living room"
            ]

#-------------------------------------------------------
# definition of callback--------------------------------
#-------------------------------------------------------

def marker(text):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = marker.TEXT_VIEW_FACING
    marker.scale.z = 0.35
    marker.color.a = 1.0
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 1
    marker.pose.orientation.w = 1.0
    marker.text = text
    marker.lifetime = rospy.Duration(2)
    
    return marker

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0
    marker.color.g = 0
    marker.color.b = 1
    marker.color.a = 0.2 # alpha to zero-->not visible

    return marker

def makeEmptyMarker( dummyBox=True ):
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position.z = marker_pos 
    marker_pos += 1
    int_marker.scale = 1
    return int_marker

def makeMenuMarker( name ):
    int_marker = makeEmptyMarker()
    int_marker.name = name

    control = InteractiveMarkerControl()

    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True

    control.markers.append( makeBox( int_marker ) )
    int_marker.controls.append(control)

    server.insert( int_marker )


def mode_photo(feedback):
    text = 'take a photo...'
    rospy.loginfo(text)
    text_show = marker(text)
    pub.publish(text_show)

    bashCommand = "rosrun robutler_detection take_photo.py"
    photo_process = subprocess.Popen(bashCommand.split())#, stdout=subprocess.PIPE)


def mode_spawn(feedback):
    global h_object_last, h_location_last

    object = menu_handler.getTitle(h_object_last)
    location = menu_handler.getTitle(h_location_last)

    text = 'spawn '+str(object) + ' in ' + str(location)
    rospy.loginfo(text)
    text_show = marker(text)
    pub.publish(text_show)

    bashCommand = "rosrun robutler_bringup_psr spawn_object.py -o "+str(object) + " -l "+str(location)
    spawn_process = subprocess.Popen(bashCommand.split())



def change_check_object(object, feedback):
    global h_object_last
    print(h_object_last)
    menu_handler.setCheckState( h_object_last, MenuHandler.UNCHECKED )
    h_object_last = feedback.menu_entry_id
    menu_handler.setCheckState( h_object_last, MenuHandler.CHECKED )

    rospy.loginfo("Switching to object: " + str(object))
    menu_handler.reApply( server )
    server.applyChanges()

def change_check_location(location, feedback):
    global h_location_last

    menu_handler.setCheckState( h_location_last, MenuHandler.UNCHECKED )
    h_location_last = feedback.menu_entry_id
    menu_handler.setCheckState( h_location_last, MenuHandler.CHECKED )

    rospy.loginfo("Switching to location: " + str(location))
    menu_handler.reApply( server )
    server.applyChanges()



def moveTo(feedback, x, y, z, R, P, Y, location, goal_publisher):

    print('Called moving to ' + location)
    p = Pose()
    p.position = Point(x=x, y=y, z=z)
    q = quaternion_from_euler(R, P, Y)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    ps = PoseStamped()
    ps.pose = p
    ps.header = Header(frame_id='map', stamp=rospy.Time.now())

    print('Sending Goal move to ' + location)
    goal_publisher.publish(ps)

    # TODO know when move is finished

    try:
        result_msg = rospy.wait_for_message('/move_base/result', MoveBaseActionResult, timeout=60)
    except:
        print('Timeout waiting for moveto')
        # TODO
        return

    print('move base completed goal with result ' + str(result_msg))

def initMenu():

    # NAVIGATION
    global h_room_last
    nav_handler = menu_handler.insert( "Move to" )
    h_room_last = menu_handler.insert("kitchen", parent=nav_handler,
                                callback=partial(moveTo,
                                                 x=6.568593, y=-1.788789, z=0,
                                                 R=0, P=0, Y=-1.504141,
                                                 location='kitchen',
                                                 goal_publisher=goal_publisher))

    h_room_last = menu_handler.insert("bedroom", parent=nav_handler,
                                callback=partial(moveTo,
                                                 x=-4.409525, y=-0.182006, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.980398,
                                                 location='bedroom',
                                                 goal_publisher=goal_publisher))
    
    h_room_last = menu_handler.insert("dining_room", parent=nav_handler,
                                callback=partial(moveTo,
                                                 x=8.491702, y=1.268367, z=0,
                                                 R=-0.000003, P=0.003177, Y=-3.102068,
                                                 location='dining_room',
                                                 goal_publisher=goal_publisher))
    
    h_room_last = menu_handler.insert("gym", parent=nav_handler,
                                callback=partial(moveTo,
                                                 x=2.150781, y=4.332005, z=0,
                                                 R=-0.000005, P=0.003177, Y=0.048769,
                                                 location='gym',
                                                 goal_publisher=goal_publisher))
    
    h_room_last = menu_handler.insert("living_room", parent=nav_handler,
                                callback=partial(moveTo,
                                                 x=0.783383, y=-0.379899, z=0,
                                                 R=-0.000003, P=0.003178, Y=-1.575511,
                                                 location='living_room',
                                                 goal_publisher=goal_publisher))

    # DETECTION
    photo_handler = menu_handler.insert( "take a photo" , callback = mode_photo)

    # SPAWN OBJECTS
    spawn_handler = menu_handler.insert( "spawn object" )

    global h_object_last
    spawn_obj_handler = menu_handler.insert( 'object', parent=spawn_handler)
    for object in objects:
        h_object_last = menu_handler.insert( str(object), parent = spawn_obj_handler, callback=partial(change_check_object, object))
        menu_handler.setCheckState( h_object_last, MenuHandler.UNCHECKED)
    # check the very last entry
    menu_handler.setCheckState( h_object_last, MenuHandler.CHECKED )

    global h_location_last
    spawn_loc_handler = menu_handler.insert( 'location', parent=spawn_handler)
    for location in locations:
        h_location_last = menu_handler.insert( str(location), parent = spawn_loc_handler, callback=partial(change_check_location, location))
        menu_handler.setCheckState( h_location_last, MenuHandler.UNCHECKED)
    # check the very last entry
    menu_handler.setCheckState( h_location_last, MenuHandler.CHECKED )

    spawn_spawn_handler = menu_handler.insert('spawn!',parent = spawn_handler, callback=mode_spawn)




if __name__=="__main__":
    rospy.init_node("mission_manager")
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    server = InteractiveMarkerServer("mission")

    initMenu()
    
    makeMenuMarker( "marker_robot" )

    menu_handler.apply( server, "marker_robot" )
    server.applyChanges()

    rospy.spin()
