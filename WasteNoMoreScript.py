ip_address = 'localhost' # Enter your IP Address here
project_identifier = 'P3B' # Enter the project identifier i.e. P3A or P3B

# SERVO TABLE CONFIGURATION
short_tower_angle = 315 # enter the value in degrees for the identification tower 
tall_tower_angle = 90 # enter the value in degrees for the classification tower
drop_tube_angle = 180 # enter the value in degrees for the drop tube. clockwise rotation from zero degrees

# BIN CONFIGURATION
# Configuration for the colors for the bins and the lines leading to those bins.
# Note: The line leading up to the bin will be the same color as the bin 

bin1_offset = 0.2 # offset in meters
bin1_color = [1,0,0] # e.g. [1,0,0] for red
bin1_metallic = False

bin2_offset = 0.2
bin2_color = [0,1,0]
bin2_metallic = False

bin3_offset = 0.2
bin3_color = [0,0,1]
bin3_metallic = False

bin4_offset = 0.2
bin4_color = [1,1,1]
bin4_metallic = False
#--------------------------------------------------------------------------------
import sys
sys.path.append('../')
from Common.simulation_project_library import *

hardware = False
if project_identifier == 'P3A':
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    configuration_information = [table_configuration, None] # Configuring just the table
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
else:
    table_configuration = [short_tower_angle,tall_tower_angle,drop_tube_angle]
    bin_configuration = [[bin1_offset,bin2_offset,bin3_offset,bin4_offset],[bin1_color,bin2_color,bin3_color,bin4_color],[bin1_metallic,bin2_metallic, bin3_metallic,bin4_metallic]]
    configuration_information = [table_configuration, bin_configuration]
    QLabs = configure_environment(project_identifier, ip_address, hardware,configuration_information).QLabs
    table = servo_table(ip_address,QLabs,table_configuration,hardware)
    arm = qarm(project_identifier,ip_address,QLabs,hardware)
    bot = qbot(0.1,ip_address,QLabs,project_identifier,hardware)

from time import sleep
import random
## Workflow 
# 
# dispense/detect container
# determine destination
# determine if bot can go
# send bot
# return bot

# Defining global variables of criticial locations
containerServoCoords = [0.63, 0.0, 0.28]
homeContracedCoords = [0.284, 0.0, 0.461]
retractPushCoords = [0.015, -0.284, 0.461]
qBotCoords = [.023, -0.5, 0.48]
raiseAboveCoords = [0.023, -0.292, 0.48]



def dispense_container():
    # Create randomly generated container, displaying properties
    containerID = random.randint(1, 6)
    containerMaterial, containerMass, containerDestination = table.dispense_container(containerID, True)

    return containerMaterial, containerMass, containerDestination 

def load_container():
    # Go to servo table and pick up container
    arm.move_arm(containerServoCoords[0], containerServoCoords[1], containerServoCoords[2])
    sleep(1)
    arm.control_gripper(35)
    sleep(1)

    # Retract to home position
    arm.move_arm(homeContracedCoords[0], homeContracedCoords[1], homeContracedCoords[2])
    sleep(1)

    # Go to qBot and deliver container
    arm.move_arm(retractPushCoords[0], retractPushCoords[1], retractPushCoords[2])
    sleep(1)
    arm.move_arm(raiseAboveCoords[0], raiseAboveCoords[1], raiseAboveCoords[2])
    sleep(1)
    arm.move_arm(qBotCoords[0], qBotCoords[1], qBotCoords[2])
    sleep(1)

    # Release gripper
    arm.control_gripper(-35)
    sleep(1)

    # Bring arm back
    arm.move_arm(retractPushCoords[0], retractPushCoords[1], retractPushCoords[2])
    sleep(.5)

    # Send arm back home
    arm.home()


def move_robot(dispensed_container): # start moving bot to correct bin
    # Activate line following sensor
    bot.activate_line_following_sensor()
    start_position = bot.position()

    # Get rgb values of desired bin, will use color sensor to compare
    target_colour = []
    if dispensed_container == "Bin01": 
        target_colour = bin1_color
    elif dispensed_container == "Bin02":
        target_colour = bin2_color
    elif dispensed_container == "Bin03":
        target_colour = bin3_color
    elif dispensed_container == "Bin04":
        target_colour = bin4_color

    # Activate ultrasonic sensor and color sensor
    bot.activate_ultrasonic_sensor()
    read_distance = bot.read_ultrasonic_sensor()
    bot.activate_color_sensor()
    
    # Trace yellow line until qbot is close enough to desired bin
    done = False
    while done == False:
        # Get current position of bot
        read_distance = bot.read_ultrasonic_sensor()
        
        # Adjust qBot speed based on yellow line positioning
        if bot.line_following_sensors() == [1, 1]: # qBot is on yellow line
            bot.set_wheel_speed([0.1, 0.1])
        elif bot.line_following_sensors() == [1, 0]: # qBot is to the left of the line
            bot.set_wheel_speed([0.06, 0.1])
        elif bot.line_following_sensors() == [0, 1]: #qBot is to the right of the line
            bot.set_wheel_speed([0.1, 0.06])
        elif bot.line_following_sensors() == [0, 0]: #qBOtot is completely lost from the line
            bot.set_wheel_speed([0.0, 0.05])
            print("Lost the line")
        
        # Check to see if the qBot is close to a bin
        if read_distance < 0.10:
            read_colour = bot.read_color_sensor()[0]
            if read_colour == target_colour: # Check to see if the qBot is at the correct bin (using the color sensor)
                 bot.stop()
                 dump_containers(dispensed_container, start_position)
                 done = True



def dump_containers(dispensed_container, start_position): #dump containers into correct bin
    bot.forward_distance(0.05)
    # Adjust bot location
    sleep(2)
    bot.deactivate_color_sensor()
    
    # Dump container in 
    bot.activate_linear_actuator()
    sleep(.1)
    bot.rotate_hopper(45)
    sleep(.5)
    bot.rotate_hopper(90)
    sleep(2)
    bot.rotate_hopper(0)
    bot.deactivate_linear_actuator()
    
    # Return the qBot to its home position
    return_home(dispensed_container, start_position)


def return_home(dispensed_container, start_position): #bring the bot back home 
    # Check current position, and 
    current_position = bot.position()
    home_check = False
    
    # Trace yellow line until reaching home
    while home_check == False:
        if bot.line_following_sensors() == [1, 1]:
            bot.set_wheel_speed([0.1, 0.1])
        elif bot.line_following_sensors() == [1, 0]:
            bot.set_wheel_speed([0.06, 0.1])
        elif bot.line_following_sensors() == [0, 1]:
            bot.set_wheel_speed([0.1, 0.06])
        elif bot.line_following_sensors() == [0, 0]:
            bot.set_wheel_speed([0.5, 0.2])
            print("Lost the line")
            
        # Get the current position of the bot and check it against the original home position
        current_position = bot.position()
        if abs(current_position[0] - start_position[0]) <= 0.02 and abs(current_position[1] - start_position[1]) <= 0.02 and abs(current_position[2] - start_position[2]) <= 0.02:
            home_check = True
    # Deactivate the sensor and stop the bot
    bot.deactivate_line_following_sensor()
    bot.stop()





def main():
    # Determine intial variable for 
    workflow = False
    containerPlaced = False # Used to determine if container is already placed on servo table
    
    while workflow == False:
        # Create variables to evaluate condition of loading qBot
        qbotDestination = ""
        qbotMass = 0
        numOfContainers = 0

        # Continual loop for loading qbot until conditions are met
        done = False 
        while done == False:
            
            # If there is already a container on the servo table, ie the first iteration of the second cycle, no container should be dispensed
            if containerPlaced == False:
                # Dispense container and retrieve destination
                containerMaterial, containerMass, containerDestination = dispense_container()
            # As the contianer from the previous iteration is taken away, there is no longer a container on the servo table, and will need to dispense one on the next iteration
            containerPlaced = False
            
            # Check to see if the desination of the qBot matches the current container
            if qbotDestination == "" or containerDestination == qbotDestination:
                # if this is the first container, the qbot will go whever the destination for this 
                if qbotDestination == "":
                    qbotDestination = containerDestination
                if qbotMass < 90.00 and numOfContainers < 3: 
                        # Update variables that track space eligibility of container
                        qbotMass += containerMass
                        numOfContainers += 1
                        # Move container to pick up location
                        load_container()
                else:
                    # If the hopper meets the conditions, then release the the qBot for drop-off
                    move_robot(qbotDestination)
                    containerPlaced = True # Container still remaning on servo table
                    done = True
                    

            else: 
                # If the dispensed container is not of the same desitnation as the qBot containers, move qBot
                move_robot(qbotDestination)
                containerPlaced = True # Container still remaning on servo table
                done = True
                
            


    

main()



    

    

