import numpy as np
import matplotlib.pyplot as plt
from library.visualize_mobile_robot import sim_mobile_robot

# Constants and Settings
Ts = 0.01 # Update simulation every 10ms
t_max = np.pi # total simulation duration in seconds
# Set initial state
init_state = np.array([-1., 0., np.pi/2]) # px, py, theta
IS_SHOWING_2DVISUALIZATION = True

# Define Field size for plotting (should be in tuple)
field_x = (-2.5, 2.5)
field_y = (-2, 2)

# IMPLEMENTATION FOR THE CONTROLLER
#---------------------------------------------------------------------
def compute_control_input(desired_state, robot_state, current_time):
    # Feel free to adjust the input and output of the function as needed.
    # And make sure it is reflected inside the loop in simulate_control()

    delta_x = desired_state[0] - robot_state[0]
    delta_y = desired_state[1] - robot_state[1]
    desired_angle = np.arctan2(delta_y, delta_x)

    error = desired_angle - robot_state[2]

    if error > np.pi:
        error -= 2 * np.pi
    elif error < -np.pi:
        error += 2 * np.pi

    Kp = 1.0 
    omega = Kp * error

    v = 0 if np.linalg.norm(desired_state[:2] - robot_state[:2]) < 0.05 else 1

    return np.array([v, omega])

# MAIN SIMULATION COMPUTATION
#---------------------------------------------------------------------
def simulate_control():
    sim_iter = round(t_max/Ts) # Total Step for simulation

    # Initialize robot's state (Single Integrator)
    robot_state = init_state.copy() # numpy array for [px, py, theta]
    desired_state = np.array([-1., 1., 0.]) # numpy array for goal / the desired [px, py, theta]

    # Store the value that needed for plotting: total step number x data length
    state_history = np.zeros( (sim_iter, len(robot_state)) ) 
    goal_history = np.zeros( (sim_iter, len(desired_state)) ) 
    input_history = np.zeros( (sim_iter, 2) ) # for [vlin, omega] vs iteration time

    if IS_SHOWING_2DVISUALIZATION: # Initialize Plot
        # sim_visualizer = sim_mobile_robot( 'omnidirectional' ) # Omnidirectional Icon
        sim_visualizer = sim_mobile_robot( 'unicycle' ) # Unicycle Icon
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
        # using discrete-time model of UNICYCLE model
        theta = robot_state[2]
        B = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])
        robot_state = robot_state + Ts*(B @ current_input) # will be used in the next iteration
        robot_state[2] = ( (robot_state[2] + np.pi) % (2*np.pi) ) - np.pi # ensure theta within [-pi pi]

        # Update desired state if we consider moving goal position
        #desired_state = desired_state + Ts*(-1)*np.ones(len(robot_state))

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
    ax.set(xlabel="t [s]", ylabel="control input")
    plt.legend()
    plt.grid()

    # Plot historical data of state
    fig3 = plt.figure(3)
    ax = plt.gca()
    ax.plot(t, state_history[:,0], label='px [m]')
    ax.plot(t, state_history[:,1], label='py [m]')
    ax.plot(t, goal_history[:,0], ':', label='goal px [m]')
    ax.plot(t, goal_history[:,1], ':', label='goal py [m]')
    ax.set(xlabel="t [s]", ylabel="state")
    plt.legend()
    plt.grid()

    # Plot time series of control input u
    fig4 = plt.figure(4)
    plt.plot(t, state_history[:, 0], label='u_x')
    plt.plot(t, state_history[:, 1], label='u_y')
    plt.xlabel('Time [s]')
    plt.ylabel('Control Input')
    plt.title('Time series of Control Input u')
    plt.legend()
    plt.grid()

    # Plot time series of error x_d - x
    fig5 = plt.figure(5)
    plt.plot(t, goal_history[:, 0] - input_history[:, 0], label='error_x')
    plt.plot(t, goal_history[:, 1] - input_history[:, 1], label='error_y')
    plt.xlabel('Time [s]')
    plt.ylabel('Error')
    plt.title('Time series of Error x_d - x')
    plt.legend()
    plt.grid()


    # Plot state trajectory compared to the desired trajectory
    fig6 = plt.figure(6)
    plt.plot(state_history[:, 0], state_history[:, 1], label='Robot Trajectory')
    plt.plot(goal_history[:, 0], state_history[:, 1], label='Desired Trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('State Trajectory compared to Desired Trajectory')
    plt.legend()
    plt.grid(True)

    plt.show()
