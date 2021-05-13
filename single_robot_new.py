#####   All Robots  #####

##########################################################################################################################
##### Provides the time and wheel velocities for a single robot to travel from a start position to a goal position   #####
##########################################################################################################################

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as img
import matplotlib.patches as patches
import matplotlib.axes as ax

print()

def elu_dist(p1, p2):
	return (np.sqrt((np.square(p2[0] - p1[0]) + np.square(p2[1] - p1[1]))))	

def get_waypoints(start_position, goal_position, samples):
    slope = (goal_position[1] - start_position[1]) / (goal_position[0] - start_position[0])
    intercept = start_position[1] - slope * start_position[0]
    theta = np.arctan(slope)

    x = np.linspace(start_position[0], goal_position[0], len(samples))
    
    for var_x in x:
        var_y = slope*var_x + intercept
        lp = plt.Circle((var_x, var_y), 10, color = 'blue')
        ax.add_artist(lp)
        var_x = round(var_x, 2)
        var_y = round(var_y, 2)
        waypoints.append([var_x, var_y])
        
    return(waypoints, theta)

def ikine(d, r, theta_dot, x_dot, y_dot):
    h_mat = [[-d/r, 1/r, 0],
            [-d/r, -1/(2*r), -(np.sin(np.pi/3)/r)], 
            [-d/r, -1/(2*r), (np.sin(np.pi/3)/r)]
            ]
    robot_vel = [[theta_dot], [x_dot], [y_dot]]

    wheel_vel = np.dot(h_mat, robot_vel)
    return(wheel_vel)

def plot_robot(position):
    
    center = position
    center_dist = 55 
    robot_rad = 85
    
    wheel1 = [center[0], center[1] +55]
    wheel2 = [center[0] + 47.6, center[1] - 27.5]
    wheel3 = [center[0] - 47.6, center[1] - 27.5]

    r = plt.Circle((center), robot_rad, color = 'white')
    ax.add_artist(r)
    w1 = plt.Circle((wheel1), 19, color = 'black')
    ax.add_artist(w1)
    w2 = plt.Circle((wheel2), 19, color = 'black')
    ax.add_artist(w2)
    w3 = plt.Circle((wheel3), 19,  color = 'black')
    ax.add_artist(w3)

################
##### Main #####
################

# Plotting Arena
arena = img.imread("arena.png")
figure, ax = plt.subplots(1)

# Provide Start and Goal Position in map
start_position = [1500, 500]
goal_position = [500, 500]
sample_time = 0.2

var_x = []
var_y = []
waypoints = []

dist = elu_dist(start_position, goal_position) #mm

robot_max_vel = 200 #0.2m/s
max_time = dist/robot_max_vel
tarr = np.arange(0,max_time,sample_time)
print("Time Required at Max Velocity = " , max_time)
print()

waypoints , theta = get_waypoints(start_position, goal_position, tarr)

x_dot = np.cos(theta)
y_dot = np.sin(theta)

r_vel_x = np.append(np.append(np.linspace(0, x_dot, 5),np.linspace(x_dot, x_dot, len(waypoints) - 10)),np.linspace(x_dot, 0, 5))
r_vel_y = np.append(np.append(np.linspace(0, y_dot, 5),np.linspace(y_dot, y_dot, len(waypoints) - 10)),np.linspace(y_dot, 0, 5))
print(r_vel_x, r_vel_y)
print(len(r_vel_x), len(r_vel_y), len(tarr), len(waypoints))

for waypoint, t, ax_x, ax_y in zip(waypoints, tarr, r_vel_x, r_vel_y):
    print("For Time", t)
    print(ax_x)
    robot_pose = [[0], [waypoint[0]], [waypoint[1]]]
    print("The Robot Pose is ", robot_pose)
    wheel_vel = ikine(55, 85, 0, ax_x, ax_y)
    print("The wheel velocities are \n ", wheel_vel)

print()

#for ax_x, ax_y in zip(x_dot, y_dot):
#    wheel_vel = ikine(55, 85, 0, ax_x, a_y)
#    print("The wheel velocities are \n ", wheel_vel)

#wheel_vel = ikine(55, 85, 0, x_dot, y_dot)
#print("The wheel velocities are \n ", wheel_vel)



#robot_pose = get_robot_pose(int_waypoints, theta)
#print(robot_pose)

#########################
##### Visualization #####
#########################

plot_robot(start_position)
plot_robot(goal_position)

sp = plt.Circle(start_position, 10)
ax.add_artist(sp)
gp = plt.Circle(goal_position, 10)
ax.add_artist(gp)

ax.imshow(arena)
plt.gca().invert_yaxis()
plt.show()