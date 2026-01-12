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
    runloop.run(
        move_attachment_1(-100, 100),
        move_attachment_2(100, 100),
        move_forward_cm(63.5, 700, 500)
    )
    await runloop.sleep_ms(100)
    await gyro_turn_absolute(90, 150)
    await move_forward_cm(10, 300, 300)
    await runloop.sleep_ms(100)
    await move_forward_cm(-10, 200, 200)
    await runloop.sleep_ms(10)
    await gyro_turn_absolute(90, 150)
    #await move_forward_ds(13, 100, 100)
    #await runloop.sleep_ms(100)

    await move_attachment_2(-130, 10000, 30000)
    await move_attachment_2(130, 10000, 30000)
    await runloop.sleep_ms(50)

    # Do Mission 8: Silo
    for i in range(3):
        await move_attachment_2(-180, 10000, 30000)
        await runloop.sleep_ms(50)
        await move_attachment_2(130, 10000, 30000)
        await runloop.sleep_ms(50)
    await move_attachment_2(130, 1000, 10000)

    # Move to Mission 6: Forge
    await move_forward_cm(1, 100, 100)
    await gyro_turn_absolute(10)
    runloop.run(
        move_forward_cm(10, 300, 300),
        move_attachment_2(130, 10000, 30000)
    )

    # Do Mission 6: Forge
    await gyro_turn_absolute(-55, 150)
    await runloop.sleep_ms(50)

    # Move to Mission 5: Who Lived Here?
    await move_forward_cm(2)
    await gyro_turn_absolute(-88)

    # Do Mission 5: Who Lived Here?
    await move_forward_cm(3)
    await runloop.sleep_ms(100)

    # Move to Mission 10: Tip the Scales
    await move_forward_cm(-10)
    await gyro_turn_absolute(-135)
    await move_forward_cm(17)
    await runloop.sleep_ms(100)
    await gyro_turn_absolute(-45)
    await move_forward_cm(22)
    await gyro_turn_absolute(-90)
    await move_forward_cm(19)
    await gyro_turn_absolute(-175)

    # Do Mission 10: Tip the Scales
    runloop.run(
        move_attachment_2(-130),
        move_forward_cm(-9)
    )
    await runloop.sleep_ms(100)
    await move_attachment_2(130)

    # Move to Mission 9: What's on Sale
    await gyro_turn_absolute(140, 250)
    await runloop.sleep_ms(100)

    # Do Mission 9: What's on Sale
    await move_forward_cm(28, 1000, 1000)
    await runloop.sleep_ms(100)
    await gyro_turn_absolute(138)
    await move_forward_cm(25, 1000, 1000)

    # Return to Red Base
    await runloop.sleep_ms(200)
    await gyro_turn_absolute(130)
    await move_forward_cm(-50, 1000, 1000)
    await move_forward_cm(13)
    await gyro_turn_absolute(-160)
    await move_forward_cm(57, 1000, 1000)
    await gyro_turn_absolute(-110)
    await move_forward_cm(90, 1000, 1000)






async def mission_b():
    hub.light_matrix.write(mission_name[current_mission]) 
    #Add all code for mission B here

    # Move to Mission 3: Mineshaft Explorer                   
    runloop.run(
        move_attachment_2(100, 100),
        move_forward_cm(60, 700, 700),
        move_attachment_1(150, 500, 500)
    )
    await runloop.sleep_ms(100)
    await move_forward_cm(-25, 600, 600)
    runloop.run(
        move_attachment_1(-200),
        move_forward_cm(25, 500, 500)

    )
    
    await move_forward_ds(27, 200)
    await runloop.sleep_ms(50)
    await gyro_turn_absolute(90, 150)
    await move_forward_cm(40.5, 400)
    await gyro_turn_absolute(0, 150) 
    await move_forward_cm(10, 200, 200)
    await move_forward_cm(-14, 200, 200)
    await runloop.sleep_ms(200)

    runloop.run(
        move_attachment_2(-130),
        gyro_turn_absolute(12, 50, 100)
    )

    await move_forward_cm(6, 100)
    await runloop.sleep_ms(200)
    await gyro_turn_absolute(14, 50, 100)

    # Do Mission 3: Mineshaft Explorer
    runloop.run(
        move_attachment_2(130, 600),
        move_forward_cm(-4, 100, 100)
    )


    # Move to Mission 02: Map Reveal
    await move_attachment_2(100, 600)
    await gyro_turn_absolute(-90)
    await move_forward_cm(20)
    await gyro_turn_absolute(-96, 100, 100)

    runloop.run(
        move_forward_cm(7),
        move_attachment_2(60, 500)
    )

    #await gyro_turn_absolute(-90)
    await move_forward_ds(13, 100, 100)
    await runloop.sleep_ms(50)

    # Do Mission 02: Map Reveal
    await gyro_turn_absolute(-55)
    await move_forward_cm(30)




    # Return to Red Base
    await gyro_turn_absolute(-110, 1000, 1000)
    await runloop.sleep_ms(500)
    await gyro_turn_absolute(-90)
    await move_forward_cm(17)
    await gyro_turn_absolute(-165)
    await move_forward_cm(80, 1000, 1000)




    await runloop.sleep_ms(1000)


# This mission is launched second from the red area
async def mission_c():
    hub.light_matrix.write(mission_name[current_mission])
    # Add all code for Mission C here


    await gyro_turn_absolute(90)
    await move_attachment_2(130)
    await move_forward_cm(36, 500, 500)
    await gyro_turn_absolute(89, 300)
    await move_attachment_2(-130)
    await move_forward_cm(-12, 100)
    await move_attachment_2(130)
    await gyro_turn_absolute(0)
    await move_forward_cm(15, 200)
    await runloop.sleep_ms(100)
    await gyro_turn_absolute(93, 200)
    await move_forward_cm(35)
    await runloop.sleep_ms(500)
    await gyro_turn_absolute(87)

    await move_forward_cm(-30, 1000, 1000)
    await gyro_turn_absolute(50)
    await move_forward_cm(48, 1000, 1000)
    await gyro_turn_absolute(115)
    await move_forward_cm(100, 1000, 1000)


async def mission_d():

    hub.light_matrix.write(mission_name[current_mission])
    runloop.run(
        move_attachment_1(-120),
        gyro_turn_absolute(-90)
    )

    await move_forward_cm(-42)
    await gyro_turn_absolute(0)
    await move_forward_cm(67)
    await move_forward_ds(12, 200, 200)
    await runloop.sleep_ms(100)
    runloop.run(
        move_forward_cm(-2, 200, 200),
        move_attachment_1(150)
    )
    await gyro_turn_absolute(-45)
    await move_attachment_1(-150)
    await gyro_turn_absolute(0)
    await move_forward_cm(-90, 1000, 1000)

async def mission_e():

    hub.light_matrix.write(mission_name[current_mission])

    # Go to Mission 10: Scale Pan
    await move_attachment_1(-180)
    await move_forward_cm(-3.5, 100, 100)
    await gyro_turn_absolute(15, 100, 100)

    # Pull Mission 10: Scale Pan
    await move_attachment_1(180)
    await move_forward_cm(-2, 100, 100)
    runloop.sleep_ms(100)
    await move_forward_cm(-10, 100, 100)
    await move_attachment_1(-180)


    
    # Go to Mission 09: Wares
    await move_forward_cm(7)
    await gyro_turn_absolute(45)
    await move_forward_cm(16, 200, 200)

    await move_attachment_1(180)
    await move_forward_cm(-5, 100, 100)
    await runloop.sleep_ms(2000)
    await move_forward_cm(1, 100, 100)
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
    await runloop.sleep_ms(500)

    # Configure the ports to the right motors / sensors
    config_ports()

    # Reset Gyro Sensor
    motion_sensor.set_yaw_face(motion_sensor.FRONT)
    motion_sensor.reset_yaw(0)

    # Start the menu
    await menu()

runloop.run(main())


