## COMP.4500- Mobile Robotics, SPR23
## Lab #3- Adding Functionality
## Danielle Le & Matthew Bedard 
#!/usr/bin/env python3
#!c:/Python35/python3.exe -u
import asyncio
import os
import sys
import math
import time
import cv2
import numpy as np
import cozmo
from glob import glob
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes, LightCube1Id 
from statemachine import State, StateMachine

try:
    from PIL import ImageDraw, ImageFont, Image
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')

class FSMCosmo(StateMachine):
    # Define states
    searchC1 = State('Searching for cube 1', initial = True)
    searchC2 = State('Searching for cube 2')
    move = State('Move to spotted cube')
    goal = State('Goal!') 

    # Define transitions
    searchc1_to_move = searchC1.to(move)
    searchc2_to_move = searchC2.to(move)
    move_to_goal = move.to(goal)
    move_to_searchc2 = move.to(searchC2)
    goal_to_searchc2 = goal.to(searchC2)

FSM = FSMCosmo()
print(FSM.current_state.id)

cube_position = None

def handle_object_appeared(evt, **kw):
        global cube_position
        # This will be called whenever an EvtObjectAppeared is dispatched whenever an Object comes into view.
        if isinstance(evt.obj, CustomObject):
            cube_position = evt.obj.pose
            # Switch to move state
            if FSM.current_state.id == 'searchC1':
                FSM.searchc1_to_move()
                cozmo.robot.Robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
                cozmo.robot.Robot.display_oled_face_image(screen_data=face_images[1], duration_ms=1000.0)
            elif FSM.current_state.id == 'searchC2':
                FSM.searchc2_to_move()
                cozmo.robot.Robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
                cozmo.robot.Robot.display_oled_face_image(screen_data=face_images[1], duration_ms=1000.0)
            #print("Cozmo started seeing a %s" % str(evt.obj.object_type))

def handle_object_disappeared(evt, **kw):
    global cube_position
    # This will be called whenever an EvtObjectDisappeared is dispatched -
    # whenever an Object goes out of view.
    if isinstance(evt.obj, CustomObject):
        #see_cube = False
        # Switch to searchC2 state
        if FSM.current_state.id == 'move':
            FSM.move_to_searchc2()
            cozmo.robot.Robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
            cozmo.robot.Robot.display_oled_face_image(screen_data= face_images[1], duration_ms=1000.0)
        elif FSM.current_state.id == 'goal':
            FSM.goal_to_searchc2()
            cozmo.robot.Robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)

       # print("Cozmo stopped seeing a %s" % str(evt.obj.object_type))

current_directory = os.path.dirname(os.path.realpath(__file__))
searching_png = os.path.join(current_directory, "searching.png")
moving_png = os.path.join(current_directory, "moving.png")
goal_png = os.path.join(current_directory, "goal.png")

image_settings = [(searching_png, Image.BICUBIC), (moving_png, Image.NEAREST), (goal_png, Image.NEAREST)]
face_images = []
for image_name, resampling_mode in image_settings:
    image = Image.open(image_name)

    # resize to fit on Cozmo's face screen
    resized_image = image.resize(cozmo.oled_face.dimensions(), resampling_mode)

    # convert the image to the format used by the oled screen
    face_image = cozmo.oled_face.convert_image_to_screen_data(resized_image, invert_image=True)
    face_images.append(face_image)

def coordinate_transform(robot_coords,cube_coords):
    robot_deg = robot_coords.rotation.angle_z.radians
    robot_cos = math.cos(robot_deg)
    robot_sin = math.sin(robot_deg)
    translation_matrix = np.matrix([[robot_cos, -robot_sin, robot_coords.position.x],
                                    [robot_sin, robot_cos, robot_coords.position.y],
                                    [0,     0,  1]])

    inverse = translation_matrix.I
    matrix = np.matrix([[cube_coords.position.x], [cube_coords.position.y],[1]])
    x = (inverse * matrix).A[0][0]
    y = (inverse * matrix).A[1][0]
    return x,y

# Distance from robot to cube
def distance(x,y):
    x_2 = x**2
    y_2 = y**2
    dist = math.sqrt(x_2 + y_2)
    return dist

async def run(robot: cozmo.robot.Robot):
    robot.set_head_angle(cozmo.util.Angle(0)) # Tilt head straight
    robot.move_lift(4) # Moves lift up
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True
    gain, exposure, mode = 390,3,1
    robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin) # Test plays noise
    time.sleep(0.5)

    robot.add_event_handler(cozmo.objects.EvtObjectAppeared, handle_object_appeared)
    robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, handle_object_disappeared)
    await robot.world.define_custom_cube(CustomObjectTypes.CustomType00, CustomObjectMarkers.Circles2, 44, 30, 30, False)

    
    robot.display_oled_face_image(screen_data=face_images[0], duration_ms=1000.0)


    while True:
        event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   # Get camera image
        # if event.image is not None:
        #     image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)
        # if mode == 1:
        #     robot.camera.enable_auto_exposure = True
        # else:
        #     robot.camera.set_manual_exposure (exposure, 390)

        if FSM.current_state.id == 'searchC1': # If in searchc1
            await robot.drive_wheels(9, -9) # Rotate around CW to search for block using 5 ms left wheel, and right wheel
            print('State: Searching for C1')

        elif FSM.current_state.id == 'searchC2': # If in searchc2
            await robot.world.undefine_all_custom_marker_objects()
            robot.add_event_handler(cozmo.objects.EvtObjectAppeared, handle_object_appeared)
            robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, handle_object_disappeared)
            await robot.world.define_custom_cube(CustomObjectTypes.CustomType00, CustomObjectMarkers.Hexagons4, 44, 30, 30, False)
            await robot.drive_wheels(9, -9) # Rotate around CW to search for block using 5 ms left wheel, and right wheel
            print('State: Searching for C2')

        elif FSM.current_state.id == 'move': #if in move state
            print("State: Moving")
            if cube_position != None:
                x1,y1 = coordinate_transform(robot.pose, cube_position)
                
                await robot.drive_wheels(10, 10)
                if distance(x1,y1) < 120:
                    print('State: Goal')
                    FSM.move_to_goal()
                    robot.display_oled_face_image(screen_data=face_images[2], duration_ms=1000.0)
                    robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
                
        elif FSM.current_state.id == 'goal': #state is in goal
            robot.stop_all_motors() # Stop robot in place when centered 
            
if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)