import opengen as og
import matplotlib.pyplot as plt
import numpy as np

from constants import *
from solver import main as compile_solver
from interface import simulate as simulate
from interface import collision_detector
from SolverError import SolverError
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from splinterp import generate_guide_trajectory
from constants import NX, N, XG, Y0, VG, K

def get_obstacle_definition():
    # Define the initial state
    initial_state = [XG - (K + 1),  # Initial x-position
                     Y0,            # Initial y-position
                     0,             # Initial orientation (psi)
                     VG,            # Initial velocity
                     0]             # Initial steering angle

    # Generate the guide trajectory
    trajectory = np.array(generate_guide_trajectory(initial_state))
    trajectory = trajectory.reshape((N, NX))  # Reshape into (N, NX)

    # Before-turn segment
    before_turn = []
    x_start = XG - (K + 35)  # Starting point for x (x=50)
    while x_start < XG - K:   
        before_turn.append([x_start, Y0, 0, VG, 0])  
        x_start += VG * DT     

    # After-turn segment
    after_turn = []
    y_start = Y0 + K  # Starting point for y (just after the circular trajectory)
    while len(after_turn) < N:  
        after_turn.append([XG, y_start, np.pi / 2, VG, 0])  
        y_start += VG * DT    

    # Combine before-turn, during-turn, and after-turn segments
    full_trajectory = before_turn + trajectory.tolist() + after_turn

    # Convert to numpy array
    full_trajectory = np.array(full_trajectory)

    # Randomly select one point from the trajectory
    random_index = np.random.randint(0, len(full_trajectory))  # Random index
    random_point = full_trajectory[random_index, :2]  # Get (x, y) from the trajectory

    random_point = np.round(random_point, 2)  # Round x and y to 2 decimals


    return [(random_point[0],random_point[1],0,-0)]


def main():
    """a function to test the solver at different obstacle positions"""
    compile_solver()

    mng = og.tcp.OptimizerTcpManager('my_optimizers/navigation_obstacle', port=12345)

    mng.start()
    mng.ping()
    

    success_counter = 0
    tries_counter = 0
    failure_counter =0

    # Initialize success and failure counters for each segment
    success_counts = { "50-60": 0, "60-70": 0, "70-80": 0, "80-90": 0, "90-100": 0, "100": 0 }
    failure_counts = { "50-60": 0, "60-70": 0, "70-80": 0, "80-90": 0, "90-100": 0, "100": 0 }

    for i in range(300):
        tries_counter += 1
        # Initial car state: x, y, theta, velocity (v), steering angle (phi)
        x = np.array([0, 0, 0, 0, 0])

        obstacle_definition = get_obstacle_definition()

        position = np.array([obstacle_definition [0][0], obstacle_definition[0][1]])
        

        try:
            x_history, u_history, obstacle_history = simulate(mng, x, obstacle_definition) 
            
            success_counter += 1
            x_history = np.array(x_history)
            u_history = np.array(u_history)
            obstacle_history = np.array(obstacle_history)

            # Categorize by x-position
            if 50 <= position[0] < 60:
                success_counts["50-60"] += 1
            elif 60 <= position[0] < 70:
                success_counts["60-70"] += 1
            elif 70 <= position[0] < 80:
                success_counts["70-80"] += 1
            elif 80 <= position[0] < 90:
                success_counts["80-90"] += 1
            elif 90 <= position[0] < 100:
                success_counts["90-100"] += 1
            elif position[0] == 100:
                success_counts["100"] += 1

            print(f"solver success for position = {position}")
            
        except SolverError:
            print(f"SolverError: solver failed to converge for position = {position}") 
            failure_counter +=1
            
            # Categorize by x-position
            if 50 <= position[0] < 60:
                failure_counts["50-60"] += 1
            elif 60 <= position[0] < 70:
                failure_counts["60-70"] += 1
            elif 70 <= position[0] < 80:
                failure_counts["70-80"] += 1
            elif 80 <= position[0] < 90:
                failure_counts["80-90"] += 1
            elif 90 <= position[0] < 100:
                failure_counts["90-100"] += 1
            elif position[0] == 100:
                failure_counts["100"] += 1
        

    # Close the TCP connection
    mng.kill()

    print(success_counter)
    print(tries_counter)
    print(failure_counter)
    print(f"success rate: {success_counter/tries_counter*100}%")

    # Data for the pie chart
    labels = ['Success', 'Failure']
    sizes = [success_counter, failure_counter]
    colors = ['blue', 'orange']  # Blue for success, orange for failure

    # Create the bar plot
    segments = ["50-60", "60-70", "70-80", "80-90", "90-100", "100"]
    success = [success_counts[segment] for segment in segments]
    failure = [failure_counts[segment] for segment in segments]

    # Bar Plot
    plt.figure(figsize=(10, 6))

    bar_width = 0.5
    index = np.arange(len(segments))

    plt.bar(index, success, bar_width, color='blue', label='Success')
    plt.bar(index, failure, bar_width, bottom=success, color='orange', label='Failure')

    plt.xlabel('Obstacle Position Segments (x position)')
    plt.ylabel('Count')
    plt.title('Success and Failure Rates per Segment')
    plt.xticks(index, segments)
    plt.legend()

    # Save the bar plot
    plt.tight_layout()
    plt.savefig("./plots/stationary_obstacle_bar.png")
    plt.close()  # Close the bar plot figure

    # Pie Chart
    plt.figure(figsize=(6, 6))

    plt.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
    plt.title(f"Success vs Failure - Success rate: {success_counter / tries_counter * 100:.2f}%")

    # Save the pie chart
    plt.tight_layout()
    plt.savefig("./plots/stationary_obstacle_pie.png")
    plt.close()  # Close the pie chart figure


if __name__ == "__main__":
    print(f"starting test")
    main()