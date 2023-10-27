#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""A node with a ROSService Server which drops water bottles when called."""

__author__ = "Malcolm Watt"

import rospy
from drone_gazebo.srv import DropBottle, DropBottleResponse
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose


def drop_bottle(req):
    """Spawns the water bottle under the plane, effectively dropping it.

    We spawn the water bottle, then set its Pose and Velocity in a seperate
    ROSService call. The Pose and Velocity is set based on the plane's state
    when this service is called, which is aquired via a service call. If any
    of the service calls fail, we indicate this in the response and exit.

    Args:
        req: DropBottleRequest effectively an empty request.

    Returns:
        DropBottleResponse; the status, a message and name of model if success.
    """
    response = DropBottleResponse()

    # Get the link state for the plane.
    try:
        link_request = GetLinkStateRequest(link_name="plane::base_link")
        plane_state = get_link_state.call(link_request)
    except rospy.ServiceException as e:
        response.success = False
        response.message = "{}\n{}".format(e, "Failed to get plane link state.")
        return response  # Exit imediately if failed.

    bottle_pose = plane_state.link_state.pose
    bottle_twist = plane_state.link_state.twist

    # Relative pose to spawn the bottle at initially.
    initial_bottle_pose = Pose()
    initial_bottle_pose.position.x = -0.3  # Roughly under the tail.
    initial_bottle_pose.position.z = -0.15
    initial_bottle_pose.orientation.y = 0.707  # Lie flat on its side.
    initial_bottle_pose.orientation.w = 0.707

    model_name = next(model_name_gen)  # Unique name for bottle model.

    # Spawn the water bottle under the plane.
    try:
        spawn_request = SpawnModelRequest(
            model_name=model_name,
            model_xml=bottle_xml(model_name),
            initial_pose=initial_bottle_pose,
            reference_frame="plane::base_link")

        spawn_resp = spawn_service.call(spawn_request)

        response.success = True
        response.message = "Water bottle spawned successfully"
        response.model_name = model_name
    except rospy.ServiceException as e:
        response.success = False
        response.message = "{}\n{}".format(e, "Failed to spawn water bottle.")
        return response  # Exit imediately if failed.

    # Set the water bottle model velocity and pose to same as plane.
    try:
        bottle_state = ModelState()
        bottle_state.model_name = model_name
        bottle_state.pose = bottle_pose
        bottle_state.twist = bottle_twist
        set_model_request = SetModelStateRequest(model_state=bottle_state)
    except rospy.ServiceException as e:
        response.success = False
        response.message = "{}\n{}".format(e, "Failed to set bottle velocity.")

    return response


def get_model_name():
    """Generator to provide a unique water bottle name each time.

    Returns:
        String model name for the bottle.
    """
    x = 0
    while True:
        yield "bottle{}".format(x)
        x += 1


def bottle_xml(name):
    """Helper which creates the bottle XML in SDF format.

    Returns:
        Model XML.
    """
    return """
    <?xml version="1.0"?>
    <sdf version="1.4">
    <model name={name}>
    <include>
      <uri>model://water_bottle</uri>
    </include>
    </model>
    """.format(name=name)


if __name__ == "__main__":
    rospy.init_node("bottle_dropper")

    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/get_link_state")
    rospy.wait_for_service("/gazebo/set_link_state")

    spawn_service = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state",
                                         SetModelState)

    model_name_gen = get_model_name()
    rospy.Service("~drop_bottle", DropBottle, drop_bottle)

    rospy.spin()
