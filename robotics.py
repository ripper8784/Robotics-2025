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
async def move_attachment_1(degrees, velocity=360):
    await motor.run_for_degrees(ATTACHMENT_1_PORT, degrees, velocity)

# Move attachment 2 by degree
async def move_attachment_2(degrees, velocity=360):
    await motor.run_for_degrees(ATTACHMENT_2_PORT, degrees, velocity)

# Move forward by distance
async def move_forward_cm(distance_cm, velocity=360, acceleration=1000):
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

# Move forward using distance sensor - distance is in cm
#async def move_forward_ds(distance_to_object, velocity=360, acceleration=1000):
#    while True:
#        if distance_sensor.distance(DISTANCE_SENSOR_PORT) > distance_to_object*10 or distance_sensor.distance(DISTANCE_SENSOR_PORT)==-1:
#            motor_pair.move(motor_pair.PAIR_1, 0, velocity=velocity, acceleration=acceleration)
#        else:
#            motor_pair.stop(motor_pair.PAIR_1, stop=motor.COAST)
#            break
#    motor_pair.stop(motor_pair.PAIR_1)


async def move_forward_ds(distance_to_object, velocity=360, acceleration=1000):
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
            motor_pair.stop(motor_pair.PAIR_1, stop=motor.COAST)
            break

        await runloop.sleep_ms(50)# Small delay for smoother control

    motor_pair.stop(motor_pair.PAIR_1)


async def gyro_turn_left_absolute(degrees, velocity = 360, acceleration = 180):
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

        await runloop.sleep_ms(10)

    motor_pair.stop(motor_pair.PAIR_1)
    await runloop.sleep_ms(50)

async def gyro_turn_right_absolute(degrees, velocity = 360, acceleration = 180):
    target = degrees * 10

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

        await runloop.sleep_ms(10)

    motor_pair.stop(motor_pair.PAIR_1)
    await runloop.sleep_ms(50)



# This function tells the code which ports are connected to which sensors / motors
def config_ports():
    # Unpair existing pairs
    motor_pair.unpair(motor_pair.PAIR_1)
    
    # Pair ports for motor
    motor_pair.pair(motor_pair.PAIR_1, LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT)



################ MISSION CODE HERE #######################

async def mission_a():
    hub.light_matrix.write(mission_name[current_mission])
    # Add all code for Mission 1 here

    await gyro_turn_left_absolute(90, 360, 180)
    



    await runloop.sleep_ms(1000)

async def mission_b():
    hub.light_matrix.write(mission_name[current_mission])
    # Add all code for Mission 2 here
    
    
    await move_attachment_1(90)
    await runloop.sleep_ms(1000)
    await move_attachment_1(-90)


async def mission_c():

    hub.light_matrix.write(mission_name[current_mission])

    await move_forward_cm(78, 1000, 1000)
    await runloop.sleep_ms(100)
    await gyro_turn_left_absolute(-55, 150)
    await move_forward_cm(5)
    await runloop.sleep_ms(200)
    await gyro_turn_left_absolute(-88)
    await move_forward_cm(3)
    await runloop.sleep_ms(200)
    await move_forward_cm(-10)
    await gyro_turn_left_absolute(-135)
    await move_forward_cm(17)
    await runloop.sleep_ms(100)
    await gyro_turn_right_absolute(-45)
    await move_forward_cm(15)
    await gyro_turn_left_absolute(-90)
    await move_forward_cm(24)
    await gyro_turn_left_absolute(-170)
    await move_forward_cm(-5)
    await runloop.sleep_ms(100)
    await move_attachment_1(90)
    await runloop.sleep_ms(200)
    await move_attachment_1(-90)
    await runloop.sleep_ms(100)
    await gyro_turn_right_absolute(-135)
    await move_forward_cm(33, 1000)
    await gyro_turn_left_absolute(-170)
    await move_forward_cm(28, 1000)
    await runloop.sleep_ms(200)
    await gyro_turn_right_absolute(-110)
    await move_forward_cm(90, 1000)
    #await move_forward_ds(30, 1000)


    
    await runloop.sleep_ms(1000)

async def mission_d():
    hub.light_matrix.write(mission_name[current_mission])
    # Add code for a combined mission
    
    

    await runloop.sleep_ms(1000)


current_mission = 0
missions =     [mission_a, mission_b, mission_c,mission_d]
mission_name = ["A",       "B",       "C",      "D"]

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


