# ################################################################################
# # Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               #
# # Leap Motion proprietary and confidential. Not for distribution.              #
# # Use subject to the terms of the Leap Motion SDK Agreement available at       #
# # https://developer.leapmotion.com/sdk_agreement, or another agreement         #
# # between Leap Motion and you, your company or other organization.             #
# ################################################################################

# import Leap, sys, _thread, time
# from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture


# class SampleListener(Leap.Listener):
#     finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
#     bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
#     state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

#     def on_init(self, controller):
#         print("Initialized")

#     def on_connect(self, controller):
#         print("Connected")

#         # Enable gestures
#         controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
#         controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
#         controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
#         controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

#     def on_disconnect(self, controller):
#         # Note: not dispatched when running in a debugger.
#         print("Disconnected")

#     def on_exit(self, controller):
#         print("Exited")

#     def on_frame(self, controller):
#         # Get the most recent frame and report some basic information
#         frame = controller.frame()

#         print("Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
#               frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures())))

#         # Get hands
#         for hand in frame.hands:

#             handType = "Left hand" if hand.is_left else "Right hand"

#             print("  %s, id %d, position: %s" % (
#                 handType, hand.id, hand.palm_position))

#             # Get the hand's normal vector and direction
#             normal = hand.palm_normal
#             direction = hand.direction

#             # Calculate the hand's pitch, roll, and yaw angles
#             print("  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
#                 direction.pitch * Leap.RAD_TO_DEG,
#                 normal.roll * Leap.RAD_TO_DEG,
#                 direction.yaw * Leap.RAD_TO_DEG))

#             # Get arm bone
#             arm = hand.arm
#             print("  Arm direction: %s, wrist position: %s, elbow position: %s" % (
#                 arm.direction,
#                 arm.wrist_position,
#                 arm.elbow_position))

#             # Get fingers
#             for finger in hand.fingers:

#                 print("    %s finger, id: %d, length: %fmm, width: %fmm" % (
#                     self.finger_names[finger.type()],
#                     finger.id,
#                     finger.length,
#                     finger.width))

#                 # Get bones
#                 for b in range(0, 4):
#                     bone = finger.bone(b)
#                     print("      Bone: %s, start: %s, end: %s, direction: %s" % (
#                         self.bone_names[bone.type],
#                         bone.prev_joint,
#                         bone.next_joint,
#                         bone.direction))

#         # Get tools
#         for tool in frame.tools:

#             print("  Tool id: %d, position: %s, direction: %s" % (
#                 tool.id, tool.tip_position, tool.direction))

#         # Get gestures
#         for gesture in frame.gestures():
#             if gesture.type == Leap.Gesture.TYPE_CIRCLE:
#                 circle = CircleGesture(gesture)

#                 # Determine clock direction using the angle between the pointable and the circle normal
#                 if circle.pointable.direction.angle_to(circle.normal) <= Leap.PI / 2:
#                     clockwiseness = "clockwise"
#                 else:
#                     clockwiseness = "counterclockwise"

#                 # Calculate the angle swept since the last frame
#                 swept_angle = 0
#                 if circle.state != Leap.Gesture.STATE_START:
#                     previous_update = CircleGesture(controller.frame(1).gesture(circle.id))
#                     swept_angle = (circle.progress - previous_update.progress) * 2 * Leap.PI

#                 print("  Circle id: %d, %s, progress: %f, radius: %f, angle: %f degrees, %s" % (
#                         gesture.id, self.state_names[gesture.state],
#                         circle.progress, circle.radius, swept_angle * Leap.RAD_TO_DEG, clockwiseness))

#             if gesture.type == Leap.Gesture.TYPE_SWIPE:
#                 swipe = SwipeGesture(gesture)
#                 print("  Swipe id: %d, state: %s, position: %s, direction: %s, speed: %f" % (
#                         gesture.id, self.state_names[gesture.state],
#                         swipe.position, swipe.direction, swipe.speed))

#             if gesture.type == Leap.Gesture.TYPE_KEY_TAP:
#                 keytap = KeyTapGesture(gesture)
#                 print("  Key Tap id: %d, %s, position: %s, direction: %s" % (
#                         gesture.id, self.state_names[gesture.state],
#                         keytap.position, keytap.direction))

#             if gesture.type == Leap.Gesture.TYPE_SCREEN_TAP:
#                 screentap = ScreenTapGesture(gesture)
#                 print("  Screen Tap id: %d, %s, position: %s, direction: %s" % (
#                         gesture.id, self.state_names[gesture.state],
#                         screentap.position, screentap.direction))

#         if not (frame.hands.is_empty and frame.gestures().is_empty):
#             print("")

#     def state_string(self, state):
#         if state == Leap.Gesture.STATE_START:
#             return "STATE_START"

#         if state == Leap.Gesture.STATE_UPDATE:
#             return "STATE_UPDATE"

#         if state == Leap.Gesture.STATE_STOP:
#             return "STATE_STOP"

#         if state == Leap.Gesture.STATE_INVALID:
#             return "STATE_INVALID"

# def main():
#     # Create a sample listener and controller
#     listener = SampleListener()
#     controller = Leap.Controller()

#     # Have the sample listener receive events from the controller
#     controller.add_listener(listener)

#     # Keep this process running until Enter is pressed
#     print("Press Enter to quit...")
#     try:
#         sys.stdin.readline()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # Remove the sample listener when done
#         controller.remove_listener(listener)


# if __name__ == "__main__":
#     main()



# """Uses interpolation in Leap API to determine the location of hands based on 
# previous data. We use the LatestEventListener to wait until we have tracking 
# events. We delay by 0.02 seconds each frame to simulate some delay, we get a 
# frame size of the frame closest to the time we want to interpolate from and 
# then interpolate on that frame"""
# import leap
# import time
# from timeit import default_timer as timer
# from typing import Callable
# from leap.events import TrackingEvent
# from leap.event_listener import LatestEventListener
# from leap.datatypes import FrameData


# def wait_until(condition: Callable[[], bool], timeout: float = 5, poll_delay: float = 0.01):
#     start_time = timer()
#     while timer() - start_time < timeout:
#         if condition():
#             return True
#         time.sleep(poll_delay)
#     if not condition():
#         return False


# def main():
#     tracking_listening = LatestEventListener(leap.EventType.Tracking)

#     connection = leap.Connection()
#     connection.add_listener(tracking_listening)

#     with connection.open() as open_connection:
#         wait_until(lambda: tracking_listening.event is not None)
#         # ctr-c to exit
#         while True:
#             event = tracking_listening.event
#             if event is None:
#                 continue
#             event_timestamp = event.timestamp

#             target_frame_size = leap.ffi.new("uint64_t*")
#             frame_time = leap.ffi.new("int64_t*")
#             frame_time[0] = event_timestamp

#             # simulate 20 ms delay
#             time.sleep(0.02)

#             try:
#                 # we need to query the storage required for our interpolation
#                 # request, the size will depend on the number visible hands in
#                 # this frame
#                 leap.get_frame_size(open_connection, frame_time, target_frame_size)
#             except Exception as e:
#                 print("get_frame_size() failed with: ", e)
#                 continue

#             frame_data = FrameData(target_frame_size[0])
#             try:
#                 # actually interpolate and get frame data from the Leap API
#                 # this is the time of the frame plus the 20ms artificial
#                 # delay and an estimated 10ms processing time which should
#                 # get close to real time hand tracking with interpolation
#                 leap.interpolate_frame(
#                     open_connection,
#                     event_timestamp + 30000,
#                     frame_data.frame_ptr(),
#                     target_frame_size[0],
#                 )
#             except Exception as e:
#                 print("interpolate_frame() failed with: ", e)
#                 continue

#             event = TrackingEvent(frame_data)
#             print(
#                 "Frame ",
#                 event.tracking_frame_id,
#                 " with ",
#                 len(event.hands),
#                 "hands with a delay of ",
#                 leap.get_now() - event.timestamp,
#             )
#             for hand in event.hands:
#                 hand_type = "left" if str(hand.type) == "HandType.Left" else "right"
#                 print(
#                     f"Hand id {hand.id} is a {hand_type} hand with position ({hand.palm.position.x}, {hand.palm.position.y}, {hand.palm.position.z})."
#                 )


# if __name__ == "__main__":
#     main()
print("hello world")
import mediapipe 

