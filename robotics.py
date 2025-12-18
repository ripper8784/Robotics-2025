import runloop
import motor_pair
import motor
import hub
import distance_sensor
import force_sensor
import math
from hub import light_matrix
from hub import port
from hub import motion_sensor
from hub import sound
from hub import button

ATTACHMENT_1_PORT=port.A
ATTACHMENT_2_PORT=port.D
DISTANCE_SENSOR_PORT=port.B
FORCE_SENSOR_PORT=port.C
LEFT_MOTOR_PORT=port.E
RIGHT_MOTOR_PORT=port.F
WHEEL_DIAMETER_CM=5.6

# Move attachment 1 by degree
async def move_attachment_1(degrees, velocity=360, acceleration=10000):
    await motor.run_for_degrees(ATTACHMENT_1_PORT, degrees, velocity, acceleration = acceleration)

# Move attachment 2 by degree
async def move_attachment_2(degrees, velocity=360, acceleration=10000):
    await motor.run_for_degrees(ATTACHMENT_2_PORT, degrees, velocity, acceleration=acceleration)

# Move forward by distance
async def move_forward_cm(distance_cm, velocity=600, acceleration=1000):
    wheel_circumference = math.pi*WHEEL_DIAMETER_CM
    degrees_to_move = int(distance_cm/wheel_circumference*360)
    await motor_pair.move_for_degrees(motor_pair.PAIR_1, degrees_to_move, 0, velocity=velocity, acceleration=acceleration)
    await runloop.sleep_ms(10)# Small delay for smoother control


# Move forward using force sensor
async def move_forward_fs(velocity=360, acceleration=1000):
    while True:
        if force_sensor.pressed(FORCE_SENSOR_PORT):
            sound.beep();
            motor_pair.stop(motor_pair.PAIR_1)
        else:
            motor_pair.move(motor_pair.PAIR_1, 0, velocity=velocity, acceleration=acceleration)
            break
    motor_pair.stop(motor_pair.PAIR_1)


async def move_forward_ds(distance_to_object, velocity=500, acceleration=1000):
    min_velocity = 100# Minimum velocity to keep moving

    while True:
        current_distance = distance_sensor.distance(DISTANCE_SENSOR_PORT)

        # If sensor can't read or distance is much greater than target, move at full speed
        if current_distance == -1 or current_distance > (distance_to_object + 20) * 10:
            motor_pair.move(motor_pair.PAIR_1, 0, velocity=velocity, acceleration=acceleration)
        # If we're close to target, slow down proportionally
        elif current_distance > distance_to_object * 10:
            # Calculate proportional velocity based on distance remaining
            distance_remaining = current_distance - (distance_to_object * 10)
            # Scale velocity: closer to target = slower speed
            proportional_velocity = max(min_velocity, int(velocity * distance_remaining / 200))
            motor_pair.move(motor_pair.PAIR_1, 0, velocity=proportional_velocity, acceleration=acceleration)
        else:
            # We've reached the target distance
            motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)
            break

        await runloop.sleep_ms(50)# Small delay for smoother control

    motor_pair.stop(motor_pair.PAIR_1)

async def move_backward_ds(distance_to_object, velocity=500, acceleration=1000):
    min_velocity = 100# Minimum velocity to keep moving

    while True:
        current_distance = distance_sensor.distance(DISTANCE_SENSOR_PORT)

        # If sensor can't read or distance is much less than target, move at full speed backward
        if current_distance == -1 or current_distance < (distance_to_object - 20) * 10:
            motor_pair.move(motor_pair.PAIR_1, 0, velocity=-velocity, acceleration=acceleration)
        # If we're close to target, slow down proportionally
        elif current_distance < distance_to_object * 10:
            # Calculate proportional velocity based on distance remaining
            distance_remaining = (distance_to_object * 10) - current_distance
            # Scale velocity: closer to target = slower speed
            proportional_velocity = max(min_velocity, int(velocity * distance_remaining / 200))
            motor_pair.move(motor_pair.PAIR_1, 0, velocity=-proportional_velocity, acceleration=acceleration)
        else:
            # We've reached the target distance
            motor_pair.stop(motor_pair.PAIR_1, stop=motor.BRAKE)
            break

        await runloop.sleep_ms(50)# Small delay for smoother control

    motor_pair.stop(motor_pair.PAIR_1)


async def gyro_turn_absolute(degrees, velocity = 500, acceleration = 180):
    target = degrees * -10

    while True:
        current = motion_sensor.tilt_angles()[0]

        # Handle 180/-180 wraparound for current reading
        if current > 1800:
            current -= 3600
        elif current < -1800:
            current += 3600

        # Calculate the shortest angular difference
        diff = target - current
        if diff > 1800:
            diff -= 3600
        elif diff < -1800:
            diff += 3600

        # Stop if we're close enough
        if abs(diff) <= 50:
            break

        # Turn in the direction of the shortest path
        if diff > 0:
            motor_pair.move(motor_pair.PAIR_1, -100, velocity=velocity, acceleration=acceleration)# Turn left
        else:
            motor_pair.move(motor_pair.PAIR_1, 100, velocity=velocity, acceleration=acceleration)# Turn right

        await runloop.sleep_ms(5)

    motor_pair.stop(motor_pair.PAIR_1)
    await runloop.sleep_ms(10)


# This function tells the code which ports are connected to which sensors / motors
def config_ports():
    # Unpair existing pairs
    motor_pair.unpair(motor_pair.PAIR_1)
    
    # Pair ports for motor
    motor_pair.pair(motor_pair.PAIR_1, LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT)



################ MISSION CODE HERE #######################

# This mission is launched first from the blue area
async def mission_a():
    hub.light_matrix.write(mission_name[current_mission])
    # Add all code for Mission 1 here

    # Move to Mission 8: Silo
    await move_forward_cm(63, 1000, 1000)
    await gyro_turn_absolute(90, 150)
    await move_forward_cm(10, 300, 300)
    await runloop.sleep_ms(10)
    await move_forward_cm(-10.5, 200, 200)
    await runloop.sleep_ms(10)
    #await move_forward_ds(13, 100, 100)
    #await runloop.sleep_ms(100)

    await move_attachment_2(-200, 20000, 30000)
    await move_attachment_2(100, 20000, 30000)
    await runloop.sleep_ms(50)

    # Do Mission 8: Silo
    for i in range(4):
        await move_attachment_2(-180, 1000, 30000)
        await runloop.sleep_ms(50)
        await move_attachment_2(180, 1000, 30000)
        await runloop.sleep_ms(50)
    await move_attachment_2(100, 1000, 10000)

    # Move to Mission 6: Forge
    await move_forward_cm(1, 100, 100)
    await gyro_turn_absolute(10)
    await move_forward_cm(11)

    # Do Mission 6: Forge
    await gyro_turn_absolute(-55, 150)
    await runloop.sleep_ms(50)

    # Move to Mission 5: Who Lived Here?
    await move_forward_cm(6)
    await gyro_turn_absolute(-88)

    # Do Mission 5: Who Lived Here?
    await move_forward_cm(3)
    await runloop.sleep_ms(100)

    # Move to Mission 10: Tip the Scales
    await move_forward_cm(-10)
    await gyro_turn_absolute(-135)
    await move_forward_cm(17)
    await runloop.sleep_ms(50)
    await gyro_turn_absolute(-45)
    await move_forward_cm(16)
    await gyro_turn_absolute(-90)
    await move_forward_cm(23)
    await gyro_turn_absolute(-170)
    await move_forward_cm(-5)
    await runloop.sleep_ms(100)

    # Do Mission 10: Tip the Scales
    await move_attachment_2(-200)
    await runloop.sleep_ms(100)
    await move_forward_cm(-5)
    await runloop.sleep_ms(200)
    await move_attachment_2(200)
    await runloop.sleep_ms(200)

    # Move to Mission 9: What's on Sale
    await gyro_turn_absolute(137, 250)
    await runloop.sleep_ms(200)

    # Do Mission 9: What's on Sale
    await move_forward_cm(28, 600, 500)
    await runloop.sleep_ms(200)

    # Return to Red Base
    await gyro_turn_absolute(130)
    await runloop.sleep_ms(200)
    await move_forward_cm(-50, 1000, 1000)
    await move_forward_cm(13)
    await gyro_turn_absolute(-160)
    await move_forward_cm(60, 1000, 1000)
    await gyro_turn_absolute(-100)
    await move_forward_cm(90, 1000, 1000)




# This mission is launched second from the red area
async def mission_b():
    hub.light_matrix.write(mission_name[current_mission])
    # Add all code for Mission B here


    await move_attachment_2(180)
    await move_forward_cm(33)
    await move_attachment_2(-180)
    await move_forward_cm(-12, 100)
    await move_attachment_2(180)
    await gyro_turn_absolute(-90)
    await move_forward_cm(15, 200)
    await gyro_turn_absolute(3, 200)
    await move_forward_cm(35)
    await runloop.sleep_ms(500)
    await gyro_turn_absolute(-3)
    await move_forward_cm(-60, 1000, 1000)




    return

    # Move to Mission 1: Surface Brushing
    await move_forward_cm(52, 300, 300)
    await runloop.sleep_ms(50)
    await move_forward_cm(20)
    await gyro_turn_absolute(0, 50, 50)
    await runloop.sleep_ms(100)
    await move_forward_cm(-10, 100, 100)
    await move_forward_cm(-50, 1000, 1000)

    # Do Mission 2: Map Reveal



async def mission_c():
    hub.light_matrix.write(mission_name[current_mission]) 

    # Move to Mission 3: Mineshaft Explorer                   
    await move_attachment_2(100, 100)
    await move_forward_cm(65, 500, 500)
    await move_forward_ds(27, 200)
    await runloop.sleep_ms(50)
    await gyro_turn_absolute(90, 150)
    await move_forward_cm(33, 400)
    await gyro_turn_absolute(0, 150) 
    await move_forward_cm(10, 200, 200)
    await move_forward_cm(-19, 200, 200)
    await runloop.sleep_ms(200)
    await gyro_turn_absolute(10, 50, 100)
    await move_attachment_2(-180)

    await move_forward_cm(11, 100)
    await runloop.sleep_ms(200)
    await gyro_turn_absolute(10, 50, 100)

    # Do Mission 3: Mineshaft Explorer
    await runloop.run_in_parallel(
        move_attachment_2(180, 500),
        move_forward_cm(-1, 100, 100)
    )


    # Move to Mission 02: Map Reveal
    await move_attachment_2(180, 500)
    await gyro_turn_absolute(-90)
    await move_forward_cm(17)
    await gyro_turn_absolute(-100, 100, 100)
    await move_forward_cm(15)
    await move_forward_ds(7, 100, 100)
    await runloop.sleep_ms(50)

    # Do Mission 02: Map Reveal
    await gyro_turn_absolute(-50)
    await move_forward_cm(25)

    # Return to Red Base
    await gyro_turn_absolute(-110, 1000, 1000)
    await runloop.sleep_ms(500)
    await gyro_turn_absolute(-90)
    await move_forward_cm(17)
    await gyro_turn_absolute(-165)
    await move_forward_cm(80, 1000, 1000)




    await runloop.sleep_ms(1000)




async def mission_d():

    hub.light_matrix.write(mission_name[current_mission])

    await move_forward_cm(-75, 1000, 1000)
    await move_forward_cm(75, 1000, 1000)

    await runloop.sleep_ms(1000)



async def mission_e():

    hub.light_matrix.write(mission_name[current_mission])

    await move_attachment_1(180)
    await runloop.sleep_ms(1000)
    await move_attachment_1(-180)


    await runloop.sleep_ms(1000)


current_mission = 0
missions =     [mission_a, mission_b, mission_c,mission_d,mission_e]
mission_name = ["A",       "B",       "C",      "D",      "E"]

async def menu():
    global current_mission

    while True:
        # Show the current mission number on the hub
        light_matrix.write(mission_name[current_mission])
        await runloop.sleep_ms(200)

        # Right button for next mission
        if button.pressed(button.RIGHT):
            current_mission = (current_mission + 1) % len(missions)
            motion_sensor.reset_yaw(0)


        # Left button select and continue
        if button.pressed(button.LEFT):
            hub.light_matrix.write('OK')
            await runloop.until(motion_sensor.stable)
            await runloop.sleep_ms(100)
            await missions[current_mission]()


async def main():
    # Say Hi
    light_matrix.show_image(light_matrix.IMAGE_TARGET)
    await runloop.sleep_ms(2000)

    # Configure the ports to the right motors / sensors
    config_ports()

    # Reset Gyro Sensor
    motion_sensor.set_yaw_face(motion_sensor.FRONT)
    motion_sensor.reset_yaw(0)

    # Start the menu
    await menu()

runloop.run(main())


