from controller import Robot, GPS
import math
import random


TIMESTEP = 32
MAX_SPEED = 6.28

class TemporalMarkovNetwork:
    def __init__(self, num_states):
        self.num_states = num_states
        self.states = ['Avoidance', 'Forward', 'GoToGoalTurn', 'GoToGoalForward']
        self.transition_matrix = [
            [[0.8, 0.1, 0.1, 0], [0.7, 0.2, 0.1, 0], [0.6, 0.2, 0.1, 0.1], [0.5, 0.3, 0.1, 0.1]],  
            [[0.2, 0.7, 0.1, 0], [0.1, 0.8, 0.1, 0], [0.1, 0.7, 0.2, 0], [0.1, 0.6, 0.2, 0.1]],  
            [[0, 0, 0.8, 0.2], [0, 0, 0.7, 0.3], [0, 0, 0.6, 0.4], [0, 0, 0.5, 0.5]],             
            [[0, 0, 0.1, 0.9], [0, 0, 0.1, 0.8], [0, 0, 0.1, 0.7], [0, 0, 0.1, 0.6]]             
        ]
        self.current_state = 'Forward'
        self.prev_state = 'Forward'  

    def transition(self):
        transition_probabilities = self.transition_matrix[self.states.index(self.current_state)][self.states.index(self.prev_state)]
        next_state_index = random.choices(range(self.num_states), weights=transition_probabilities)[0]
        self.prev_state = self.current_state
        self.current_state = self.states[next_state_index]

def obstacle_detected(ps_values):
    
    return any(val > 75 for val in ps_values)

def limit_velocity(velocity, max_velocity):
    return min(max_velocity, max(-max_velocity, velocity))

def run_robot(robot):
    
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    
    list_ps = [robot.getDevice('ps' + str(ind)) for ind in [0, 1, 2, 3, 4, 5, 6, 7]]
    [ps.enable(TIMESTEP) for ps in list_ps]

    
    gps = robot.getDevice('gps')
    gps.enable(TIMESTEP)

    
    tmn = TemporalMarkovNetwork(num_states=4)  
    
    goal_x = 0.5  
    goal_z = -0.3  

    last_distance_to_goal = float('inf')  

    while robot.step(TIMESTEP) != -1:
        ps_values = [ps.getValue() for ps in list_ps]
        
        gps_values = gps.getValues()

        distance_to_goal = math.sqrt((goal_x - gps_values[0]) ** 2 + (goal_z - gps_values[2]) ** 2)
        print("Distance to goal:", distance_to_goal)
  
        if distance_to_goal < last_distance_to_goal:
            tmn.current_state = 'Forward'
        else:
            tmn.transition()
       
        if obstacle_detected(ps_values):
            tmn.current_state = 'Avoidance'

        left_speed = MAX_SPEED
        right_speed = MAX_SPEED

        if tmn.current_state == 'Avoidance':
            
            left_speed = -MAX_SPEED
            right_speed = MAX_SPEED

        elif tmn.current_state == 'GoToGoalTurn':
           
            left_speed = -MAX_SPEED / 2
            right_speed = MAX_SPEED

        elif tmn.current_state == 'GoToGoalForward':
            
            kp_distance = 0.1
            kp_heading = 1.0
            angle_to_goal = math.atan2(goal_z - gps_values[2], goal_x - gps_values[0])
            distance_signal = kp_distance * distance_to_goal
            heading_signal = kp_heading * angle_to_goal
            left_speed = limit_velocity(MAX_SPEED - heading_signal + distance_signal, MAX_SPEED)
            right_speed = limit_velocity(MAX_SPEED + heading_signal - distance_signal, MAX_SPEED)

        elif tmn.current_state == 'Forward':
            
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED
        
        if distance_to_goal < 0.2:
            left_speed = 0
            right_speed = 0
            print("Reached the goal!")

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

        last_distance_to_goal = distance_to_goal

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
