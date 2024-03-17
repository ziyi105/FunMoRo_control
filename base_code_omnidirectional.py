import numpy as np
import matplotlib.pyplot as plt
import math
from library.visualize_mobile_robot import sim_mobile_robot

# Constants and Settings
Ts = 0.01 # Update simulation every 10ms
t_max = np.pi * 2 # total simulation duration in seconds
# Set initial state
init_state = np.array([0., 1.5, 0.]) # px, py, theta
IS_SHOWING_2DVISUALIZATION = True
k = 3

# Define Field size for plotting (should be in tuple) 
field_x = (-2.5, 2.5)
field_y = (-2, 2)

# IMPLEMENTATION FOR THE CONTROLLER
#---------------------------------------------------------------------
def xd(t):
    return np.array([-2 * np.cos(t), np.sin(t), 0])

def xd_dot(t):
    return np.array([2 * np.sin(t), np.cos(t), 0])

def compute_control_input(desired_state, robot_state, current_time):
    # Feel free to adjust the input and output of the function as needed.
    # And make sure it is reflected inside the loop in simulate_control()

    # initial numpy array for [vx, vy, omega]
    control_input = np.array([0., 0., 0.]) 
    
    # Compute the control input
    error = desired_state - robot_state
    desired_state_dot = xd_dot(current_time)

    control_input = k * error + desired_state_dot

    return control_input


# MAIN SIMULATION COMPUTATION
#---------------------------------------------------------------------
def simulate_control():
    sim_iter = round(t_max/Ts) # Total Step for simulation

    # Initialize robot's state (Single Integrator)
    robot_state = init_state.copy() # numpy array for [px, py, theta]
    desired_state = np.array([-2., 0., 0.]) # numpy array for goal / the desired [px, py, theta]

    # Store the value that needed for plotting: total step number x data length
    state_history = np.zeros( (sim_iter, len(robot_state)) ) 
    goal_history = np.zeros( (sim_iter, len(desired_state)) ) 
    input_history = np.zeros( (sim_iter, 3) ) # for [vx, vy, omega] vs iteration time

    if IS_SHOWING_2DVISUALIZATION: # Initialize Plot
        sim_visualizer = sim_mobile_robot( 'omnidirectional' ) # Omnidirectional Icon
        #sim_visualizer = sim_mobile_robot( 'unicycle' ) # Unicycle Icon
        sim_visualizer.set_field( field_x, field_y ) # set plot area
        sim_visualizer.show_goal(desired_state)

    for it in range(sim_iter):
        current_time = it*Ts
        # record current state at time-step t
        state_history[it] = robot_state
        goal_history[it] = desired_state

        # COMPUTE CONTROL INPUT
        #------------------------------------------------------------
        current_input = compute_control_input(desired_state, robot_state, current_time)
        #------------------------------------------------------------

        # record the computed input at time-step t
        input_history[it] = current_input

        if IS_SHOWING_2DVISUALIZATION: # Update Plot
            sim_visualizer.update_time_stamp( current_time )
            sim_visualizer.update_goal( desired_state )
            sim_visualizer.update_trajectory( state_history[:it+1] ) # up to the latest data
        
        #--------------------------------------------------------------------------------
        # Update new state of the robot at time-step t+1
        # using discrete-time model of single integrator dynamics for omnidirectional robot
        robot_velocity = current_input
        robot_state[0] += Ts * robot_velocity[0]  # Update x position
        robot_state[1] += Ts * robot_velocity[1]  # Update y position
        robot_state[2] += Ts * robot_velocity[2]  # Update orientation
        robot_state[2] = ( (robot_state[2] + np.pi) % (2*np.pi) ) - np.pi # ensure theta within [-pi pi]

        # Update desired state if we consider moving goal position
        desired_state = xd(current_time)

    # End of iterations
    # ---------------------------
    # return the stored value for additional plotting or comparison of parameters
    return state_history, goal_history, input_history


if __name__ == '__main__':
    
    # Call main computation for robot simulation
    state_history, goal_history, input_history = simulate_control()


    # ADDITIONAL PLOTTING
    #----------------------------------------------
    t = [i*Ts for i in range( round(t_max/Ts) )]

    # Plot historical data of control input
    fig2 = plt.figure(2)
    ax = plt.gca()
    ax.plot(t, input_history[:,0], label='vx [m/s]')
    ax.plot(t, input_history[:,1], label='vy [m/s]')
    ax.plot(t, input_history[:,2], label='omega [rad/s]')
    ax.set(xlabel="t [s]", ylabel="control input")
    plt.legend()
    plt.grid()

    # Plot historical data of state
    fig3 = plt.figure(3)
    ax = plt.gca()
    ax.plot(t, state_history[:,0], label='px [m]')
    ax.plot(t, state_history[:,1], label='py [m]')
    ax.plot(t, state_history[:,2], label='theta [rad]')
    ax.plot(t, goal_history[:,0], ':', label='goal px [m]')
    ax.plot(t, goal_history[:,1], ':', label='goal py [m]')
    ax.plot(t, goal_history[:,2], ':', label='goal theta [rad]')
    ax.set(xlabel="t [s]", ylabel="state")
    plt.legend()
    plt.grid()

    # Plot time series of control input u
    fig4 = plt.figure(4)
    plt.plot(t, state_history[:, 0], label='u_x')
    plt.plot(t, state_history[:, 1], label='u_y')
    plt.plot(t, state_history[:, 2], label='u_theta')
    plt.xlabel('Time [s]')
    plt.ylabel('Control Input')
    plt.title('Time series of Control Input u')
    plt.legend()
    plt.grid()

    # Plot time series of error x_d - x
    fig5 = plt.figure(5)
    plt.plot(t, goal_history[:, 0] - input_history[:, 0], label='error_x')
    plt.plot(t, goal_history[:, 1] - input_history[:, 1], label='error_y')
    plt.plot(t, goal_history[:, 2] - input_history[:, 2], label='error_theta')
    plt.xlabel('Time [s]')
    plt.ylabel('Error')
    plt.title('Time series of Error x_d - x')
    plt.legend()
    plt.grid()


    # Plot state trajectory compared to the desired trajectory
    fig6 = plt.figure(6)
    plt.plot(state_history[:, 0], state_history
             [:, 1], label='Robot Trajectory')
    plt.plot([xd(t)[0] for t in np.arange(0, t_max, Ts)], [xd(t)[1] for t in np.arange(0, t_max, Ts)], label='Desired Trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('State Trajectory compared to Desired Trajectory')
    plt.legend()
    plt.grid(True)

    plt.show()
