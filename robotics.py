import runloop
import motor_pair
import motor
import hub
import distance_sensor
import force_sensor
from hub import light_matrix
from hub import port
from hub import motion_sensor
from hub import sound
from hub import button

ATTACHMENT_1_PORT=port.A
ATTACHMENT_2_PORT=port.B
DISTANCE_SENSOR_PORT=port.F
FORCE_SENSOR_PORT=port.E
LEFT_MOTOR_PORT=port.C
RIGHT_MOTOR_PORT=port.D
WHEEL_DIAMETER_CM=10

# Move forward using force sensor
async def move_forward_cm(distance_cm, velocity=360, acceleration=100):
    degrees_to_move = distance_cm/WHEEL_DIAMETER_CM*360
    motor_pair.move_for_degrees(pair=motor_pair.PAIR_1, steering=0, degrees=degrees_to_move, velocity=velocity, acceleration=acceleration)
    motor_pair.move(pair=motor_pair.PAIR_1, steering=0, velocity=velocity, acceleration=acceleration)
    motor_pair.stop(motor_pair.PAIR_1)


# Move forward using force sensor
async def move_forward_fs(velocity=360, acceleration=100):
    while True:
        if force_sensor.pressed(FORCE_SENSOR_PORT):
            sound.beep();
            motor_pair.stop(motor_pair.PAIR_1)
        else:
            motor_pair.move(pair=motor_pair.PAIR_1, steering=0, velocity=velocity, acceleration=acceleration)
            break
    motor_pair.stop(motor_pair.PAIR_1)

# Move forward using distance sensor - distance is in cm
async def move_forward_ds(distance_to_object, velocity=360, acceleration=100):
    while True:
        if distance_sensor.distance(DISTANCE_SENSOR_PORT) > distance_to_object*10 or distance_sensor.distance(DISTANCE_SENSOR_PORT)==-1:
            motor_pair.move(pair=motor_pair.PAIR_1, steering=0, velocity=velocity, acceleration=acceleration)
        else:
            motor_pair.stop(motor_pair.PAIR_1)
            break
    motor_pair.stop(motor_pair.PAIR_1)

# This function makes an accurate left turn
async def gyro_turn_left(degrees):
    motion_sensor.reset_yaw(0)
    while motion_sensor.tilt_angles()[0] < degrees*10:
        motor_pair.move(motor_pair.PAIR_1, -100)
    motor_pair.stop(motor_pair.PAIR_1)

# This function makes an accurate right turn
async def gyro_turn_right(degrees):
    motion_sensor.reset_yaw(0)
    while motion_sensor.tilt_angles()[0] > degrees*-10:
        motor_pair.move(motor_pair.PAIR_1, 100)
    motor_pair.stop(motor_pair.PAIR_1)


# This function tells the code which ports are connected to which sensors / motors
def config_ports():
    # Unpair existing pairs
    motor_pair.unpair(motor_pair.PAIR_1)
    
    # Pair ports for motor
    motor_pair.pair(motor_pair.PAIR_1, LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT)



################ MISSION CODE HERE #######################

def mission_1():
    hub.light_matrix.write(mission_name[current_mission])
    # Add all code for Mission 1 here


    runloop.sleep_ms(3000)

def mission_2():
    hub.light_matrix.write(mission_name[current_mission])
    # Add all code for Mission 2 here


    runloop.sleep_ms(3000)

def mission_3_and_4():
    hub.light_matrix.write(mission_name[current_mission])
    # Add code for a combined mission


    runloop.sleep_ms(3000)


current_mission = 0
missions =     [mission_1, mission_2, mission_3_and_4]
mission_name = ["M1",      "M2",      "M34"]

def menu():
    global current_mission

    while True:
        # Show the current mission number on the hub
        light_matrix.write(str(current_mission + 1))
        runloop.sleep_ms(200)

        # Right button for next mission
        if button.pressed(button.RIGHT):
            current_mission = (current_mission + 1) % len(missions)

        # Left button select and continue
        if button.pressed(button.LEFT):
            missions[current_mission]()
            hub.light_matrix.write('DONE')
            runloop.sleep_ms(1000)


async def main():
    # Say Hi
    await light_matrix.write(text="Hi!")

    # Configure the ports to the right motors / sensors
    config_ports()

    # Reset Gyro Sensor
    motion_sensor.set_yaw_face(up=motion_sensor.FRONT)
    motion_sensor.reset_yaw(0)

    # Start the menu
    menu()

runloop.run(main())


