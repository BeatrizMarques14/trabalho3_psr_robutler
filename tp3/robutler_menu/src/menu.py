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
from std_msgs.msg import Bool

server = None
marker_pos = 0

menu_handler = MenuHandler()

pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

rooms = ['kitchen','living_room','bedroom','gym'] 

bedroom_locations = ["on_bed", "on_bed_side_table",'on_desk','on_bedroom_chair','beside_bed','beside_ball']

kitchen_locations = ['on_kitchen_counter', 'on_kitchen_table', 'beside_counter', 'beside_table', 'in_front_of_fridge']

living_room_locations = ['on_sofa', 'on_side_table', 'on_shelf','lying_on_couch']

gym_locations = ['working_out', 'under_the_weight_bench']

objects = ['sphere_v','person_standing','blue_cube']#

missions = [
            "find a purple sphere",
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

def change_check_object_mission(object, feedback):
    global h_obj
    print(h_obj)
    menu_handler.setCheckState( h_obj, MenuHandler.UNCHECKED )
    h_obj = feedback.menu_entry_id
    menu_handler.setCheckState( h_obj, MenuHandler.CHECKED )

    rospy.loginfo("Switching to object: " + str(object))
    menu_handler.reApply( server )
    server.applyChanges()

def change_check_object_house_mission(object, feedback):
    global h_house_obj
    print(h_house_obj)
    menu_handler.setCheckState( h_house_obj, MenuHandler.UNCHECKED )
    h_house_obj = feedback.menu_entry_id
    menu_handler.setCheckState( h_house_obj, MenuHandler.CHECKED )

    rospy.loginfo("Switching to object: " + str(object))
    menu_handler.reApply( server )
    server.applyChanges()

def change_check_room(location, feedback):
    global h_room

    menu_handler.setCheckState( h_room, MenuHandler.UNCHECKED )
    h_room = feedback.menu_entry_id
    menu_handler.setCheckState( h_room, MenuHandler.CHECKED )

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




def findObjectLocation(feedback):
    global h_obj, h_room

    object = menu_handler.getTitle(h_obj)
    room = menu_handler.getTitle(h_room)

    rospy.loginfo(f"Searching for a {object} in {room}")

    if object == 'sphere_v':
        bashCommand = "rosrun robutler_detection detect_color.py violet"
    elif object == 'blue_cube':
        bashCommand = "rosrun robutler_detection detect_color.py blue"

    #FIND OBJECT IN GYM
    if room == 'gym':
        moveTo(feedback, x=3.564367, y=1.487425, z=0,R=-0.000005, P=0.003179, Y=1.721249, location=room, goal_publisher=goal_publisher)
        proc_color  = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        rospy.sleep(2)
        object_found = rospy.wait_for_message("/object_detection_result", Bool).data

    #FIND OBJECT IN BEDROOM
    elif room == 'bedroom':
        moveTo(feedback, x=-4.306904, y=-0.421522, z=0,R=-0.000007, P=0.003198, Y=1.947747, location=room, goal_publisher=goal_publisher)
        proc_color  = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        rospy.sleep(2)
        object_found = rospy.wait_for_message("/object_detection_result", Bool).data

        moveTo(feedback, x=-4.447187, y=1.385214, z=0,R=-0.000007, P=0.003198, Y=1.552910, location=room, goal_publisher=goal_publisher)
        if not object_found:
            rospy.sleep(2)
            object_found = rospy.wait_for_message("/object_detection_result", Bool).data

        moveTo(feedback, x=-7.667101, y=0.094760, z=0,R=-0.000007, P=0.003198, Y=1.776941, location=room, goal_publisher=goal_publisher)
        if not object_found:
            rospy.sleep(2)
            object_found = rospy.wait_for_message("/object_detection_result", Bool).data

        moveTo(feedback, x=-8.915124, y=-1.292291, z=0,R=-0.000007, P=0.003198, Y=-1.420993, location=room, goal_publisher=goal_publisher)

        moveTo(feedback, x=-7.725993, y=-3.202666, z=0,R=-0.000007, P=0.003198, Y=-2.037030, location=room, goal_publisher=goal_publisher)
        if not object_found:
            rospy.sleep(2)
            object_found = rospy.wait_for_message("/object_detection_result", Bool).data
        
        moveTo(feedback, x=-7.725993, y=-3.202666, z=0,R=-0.000007, P=0.003198, Y=-0.768974, location=room, goal_publisher=goal_publisher)
        if not object_found:
            rospy.sleep(2)
            object_found = rospy.wait_for_message("/object_detection_result", Bool).data
        
        moveTo(feedback, x=-2.413613, y=-1.545555, z=0,R=-0.000007, P=0.003198, Y=0.856001, location=room, goal_publisher=goal_publisher)
    
    #FIND OBJECT IN LIVING ROOM
    elif room == "living_room":

        moveTo(feedback, x=1.070897, y=-0.924668, z=0,R=-0.000005, P=0.003179, Y=-1.853565, location=room, goal_publisher=goal_publisher)
        proc_color  = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        rospy.sleep(2)
        object_found = rospy.wait_for_message("/object_detection_result", Bool).data

        moveTo(feedback, x=1.071054, y=-0.924797, z=0,R=-0.000007, P=0.003198, Y=-2.727902, location=room, goal_publisher=goal_publisher)
        if not object_found:
            rospy.sleep(2)
            object_found = rospy.wait_for_message("/object_detection_result", Bool).data
        
        moveTo(feedback, x=3.610576, y=-0.172326, z=0,R=-0.000007, P=0.003198, Y=-1.501944, location=room, goal_publisher=goal_publisher)
        moveTo(feedback, x=4.417015, y=-3.594175, z=0,R=-0.000007, P=0.003198, Y=-1.589444, location=room, goal_publisher=goal_publisher)
        if not object_found:
            rospy.sleep(2)
            object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    
    elif room == "kitchen":

        moveTo(feedback, x=6.568593, y=-1.788789, z=0,R=0, P=0, Y=-1.504141, location=room, goal_publisher=goal_publisher)
        proc_color  = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        rospy.sleep(2)
        object_found = rospy.wait_for_message("/object_detection_result", Bool).data

        moveTo(feedback, x=6.461837, y=-2.114999, z=0,R=-0.000007, P=0.003198, Y=-0.677759, location=room, goal_publisher=goal_publisher)
        if not object_found:
            rospy.sleep(2)
            object_found = rospy.wait_for_message("/object_detection_result", Bool).data

        moveTo(feedback, x=6.461837, y=-2.114999, z=0,R=-0.000007, P=0.003198, Y=-0.694105, location=room, goal_publisher=goal_publisher)
        if not object_found:
            rospy.sleep(2)
            object_found = rospy.wait_for_message("/object_detection_result", Bool).data
        
        moveTo(feedback, x=6.138995, y=-1.214863, z=0,R=-0.000007, P=0.003198, Y=-0.040767, location=room, goal_publisher=goal_publisher)
        if not object_found:
            rospy.sleep(2)
            object_found = rospy.wait_for_message("/object_detection_result", Bool).data

        moveTo(feedback, x=4.582840, y=0.194559, z=0,R=-0.000007, P=0.003198, Y=0.695034, location=room, goal_publisher=goal_publisher)
        if not object_found:
            rospy.sleep(2)
            object_found = rospy.wait_for_message("/object_detection_result", Bool).data


    if object_found:
        text = f"Found {object} in {room}!"
    else:
        text = f"{object} not found in {room}."

    text_show = marker(text)
    pub.publish(text_show)
    proc_color.terminate()


def isTableClean(feedback):
    
    moveTo(feedback, x=4.335937, y=1.164780, z=0,R=-0.000007, P=0.003198, Y=-0.006472, location='kitchen', goal_publisher=goal_publisher)

    #search for violet
    bashCommand = "rosrun robutler_detection detect_color.py violet"
    proc_color  = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    proc_color.terminate()

    #search for blue
    if not object_found:
        bashCommand = "rosrun robutler_detection detect_color.py blue"
        proc_color  = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        rospy.sleep(2)
        object_found = rospy.wait_for_message("/object_detection_result", Bool).data
        proc_color.terminate()
    
    if object_found:
        text = "Table is not clean"
    else:
        text = "Table is clean"

    text_show = marker(text)
    pub.publish(text_show)
    proc_color.terminate()

def house_objects(feedback,object):
    
    
    counter = 0
    if object == 'sphere_v':
        bashCommand = "rosrun robutler_detection detect_color.py violet"
    elif object == 'blue_cube':
        bashCommand = "rosrun robutler_detection detect_color.py blue"
    
    #BEDROOM
    moveTo(feedback, x=-4.306904, y=-0.421522, z=0,R=-0.000007, P=0.003198, Y=1.947747, location="bedroom", goal_publisher=goal_publisher)
    proc_color  = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
    

    moveTo(feedback, x=-4.447187, y=1.385214, z=0,R=-0.000007, P=0.003198, Y=1.552910, location="bedroom", goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)

    moveTo(feedback, x=-4.861541, y=0.514091, z=0,R=-0.000007, P=0.003198, Y=-2.947357, location="bedroom", goal_publisher=goal_publisher)

    moveTo(feedback, x=-7.667101, y=0.094760, z=0,R=-0.000007, P=0.003198, Y=1.776941, location="bedroom", goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)

    moveTo(feedback, x=-8.915124, y=-1.292291, z=0,R=-0.000007, P=0.003198, Y=-1.420993, location="bedroom", goal_publisher=goal_publisher)

    moveTo(feedback, x=-7.725993, y=-3.202666, z=0,R=-0.000007, P=0.003198, Y=-2.037030, location="bedroom", goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)
    
    moveTo(feedback, x=-7.725993, y=-3.202666, z=0,R=-0.000007, P=0.003198, Y=-0.768974, location="bedroom", goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)
        
    moveTo(feedback, x=-2.413613, y=-1.545555, z=0,R=-0.000007, P=0.003198, Y=0.856001, location="bedroom", goal_publisher=goal_publisher)


    #LIVING ROOM
    moveTo(feedback, x=-0.733127, y=-0.040851, z=0,R=-0.000007, P=0.003198, Y=0.040111, location="bedroom", goal_publisher=goal_publisher)

    moveTo(feedback, x=1.070897, y=-0.924668, z=0,R=-0.000005, P=0.003179, Y=-1.853565, location='living_room', goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)

    moveTo(feedback, x=1.071054, y=-0.924797, z=0,R=-0.000007, P=0.003198, Y=-2.727902, location='living_room', goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)
        
    moveTo(feedback, x=3.610576, y=-0.172326, z=0,R=-0.000007, P=0.003198, Y=-1.501944, location='living_room', goal_publisher=goal_publisher)
    moveTo(feedback, x=4.417015, y=-3.594175, z=0,R=-0.000007, P=0.003198, Y=-1.589444, location='living_room', goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)
    
    #KITCHEN
    moveTo(feedback, x=6.568593, y=-1.788789, z=0,R=0, P=0, Y=-1.504141, location='kitchen', goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)

    moveTo(feedback, x=6.461837, y=-2.114999, z=0,R=-0.000007, P=0.003198, Y=-0.677759, location='kitchen', goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)

    moveTo(feedback, x=6.461837, y=-2.114999, z=0,R=-0.000007, P=0.003198, Y=-0.694105, location='kitchen', goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)
        
    moveTo(feedback, x=6.138995, y=-1.214863, z=0,R=-0.000007, P=0.003198, Y=-0.040767, location='kitchen', goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)

    moveTo(feedback, x=4.335937, y=1.164780, z=0,R=-0.000007, P=0.003198, Y=-0.006472, location='kitchen', goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)

    #GYM
    
    moveTo(feedback, x=3.564367, y=1.487425, z=0,R=-0.000005, P=0.003179, Y=1.721249, location='gym', goal_publisher=goal_publisher)
    rospy.sleep(2)
    object_found = rospy.wait_for_message("/object_detection_result", Bool).data
    if object_found:
        counter+=1
    text = str(counter)
    text_show = marker(text)
    pub.publish(text_show)

    if counter == 0:
        text = f"{object} not found!"
    else:
        text = f"Found {counter} {object}."

    text_show = marker(text)
    pub.publish(text_show)
    proc_color.terminate()




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
                                                 x=-4.306904, y=-0.421522, z=0,
                                                 R=-0.000007, P=0.003198, Y=1.947747,
                                                 location='bedroom',
                                                 goal_publisher=goal_publisher))
    
    
    h_room_last = menu_handler.insert("gym", parent=nav_handler,
                                callback=partial(moveTo,
                                                 x=4.262566, y=1.779691, z=0,
                                                 R=-0.000005, P=0.003179, Y=1.564164,
                                                 location='gym',
                                                 goal_publisher=goal_publisher))
    
    h_room_last = menu_handler.insert("living_room", parent=nav_handler,
                                callback=partial(moveTo,
                                                 x=0.783383, y=-0.379899, z=0,
                                                 R=-0.000003, P=0.003178, Y=-1.575511,
                                                 location='living_room',
                                                 goal_publisher=goal_publisher))

    # SPAWN OBJECTS
    spawn_handler = menu_handler.insert( "spawn object" )

    global h_object_last
    spawn_obj_handler = menu_handler.insert( 'object', parent=spawn_handler)
    for object in objects:
        h_object_last = menu_handler.insert( str(object), parent = spawn_obj_handler, callback=partial(change_check_object, object))
        menu_handler.setCheckState( h_object_last, MenuHandler.UNCHECKED)
    menu_handler.setCheckState( h_object_last, MenuHandler.CHECKED )

    global h_location_last
    spawn_loc_handler = menu_handler.insert( 'location', parent=spawn_handler)
    bedroom_handler = menu_handler.insert('bedroom',parent = spawn_loc_handler)
    for location in bedroom_locations:
        h_location_last = menu_handler.insert( str(location), parent = bedroom_handler, callback=partial(change_check_location, location))
        menu_handler.setCheckState( h_location_last, MenuHandler.UNCHECKED)

    kitchen_handler = menu_handler.insert('kitchen',parent = spawn_loc_handler)
    for location in kitchen_locations:
        h_location_last = menu_handler.insert( str(location), parent = kitchen_handler, callback=partial(change_check_location, location))
        menu_handler.setCheckState( h_location_last, MenuHandler.UNCHECKED)
        

    living_room_handler = menu_handler.insert('living room',parent = spawn_loc_handler)
    for location in living_room_locations:
        h_location_last = menu_handler.insert( str(location), parent = living_room_handler, callback=partial(change_check_location, location))
        menu_handler.setCheckState( h_location_last, MenuHandler.UNCHECKED)


    gym_handler = menu_handler.insert('gym',parent = spawn_loc_handler)
    for location in gym_locations:
        h_location_last = menu_handler.insert( str(location), parent = gym_handler, callback=partial(change_check_location, location))
        menu_handler.setCheckState( h_location_last, MenuHandler.UNCHECKED)
    
    menu_handler.setCheckState( h_location_last, MenuHandler.CHECKED )

    spawn_spawn_handler = menu_handler.insert('spawn!',parent = spawn_handler, callback=mode_spawn)

    #TAKE PHOTO
    photo_handler = menu_handler.insert( "take a photo" , callback = mode_photo)

    #Missions
    missions_handler = menu_handler.insert("Missions")

    #FIND OBJECT IN LOCATION
    find_object_handler = menu_handler.insert("find object", parent=missions_handler)

    global h_obj
    find_obj_handler = menu_handler.insert("objects", parent = find_object_handler)
    for object in objects:
        h_obj = menu_handler.insert( str(object), parent = find_obj_handler, callback=partial(change_check_object_mission, object))
        menu_handler.setCheckState( h_obj, MenuHandler.UNCHECKED)
    menu_handler.setCheckState( h_obj, MenuHandler.CHECKED )

    global h_room
    find_loc_handler = menu_handler.insert('rooms', parent= find_object_handler)
    for room in rooms:
        h_room = menu_handler.insert( str(room), parent = find_loc_handler, callback=partial(change_check_room, room))
        menu_handler.setCheckState( h_room, MenuHandler.UNCHECKED)
    menu_handler.setCheckState( h_room, MenuHandler.CHECKED )

    find_handler = menu_handler.insert("Find!", parent= find_object_handler, callback=findObjectLocation)

    #IS TABLE CLEAN MISSION
    table_clean_handler = menu_handler.insert("Is table clean", parent = missions_handler, callback=isTableClean)

    objects_in_house_handler = menu_handler.insert("How many objects are in the house", parent = missions_handler)
    global h_house_obj
    h_house_obj = menu_handler.insert( str("sphere_v"), parent = objects_in_house_handler, callback=partial(house_objects, object="sphere_v"))
    h_house_obj = menu_handler.insert( str("blue_cube"), parent = objects_in_house_handler, callback=partial(house_objects, object="blue_cube"))

        
    





if __name__=="__main__":
    rospy.init_node("mission_manager")
    goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    server = InteractiveMarkerServer("mission")

    initMenu()
    
    makeMenuMarker( "marker_robot" )

    menu_handler.apply( server, "marker_robot" )
    server.applyChanges()

    rospy.spin()
