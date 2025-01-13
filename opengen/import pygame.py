import pygame
import sys
import csv
import os
import math

pygame.init()

#Screen configuration
WIDTH, HEIGHT = 1100, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Visualizador de Simulação 2D")
clock = pygame.time.Clock()

# Load images
robot_image = pygame.image.load("robot.png")
obstacle_static_image = pygame.image.load("armazem.png")
road_image = pygame.image.load("road.png")
road_image2 = road_image
obstacle_dynamic_image = pygame.image.load("Car.png")

#Objects dimentions
robot_length=10*2.6
robot_width=10
cars_length=10*2.2
cars_width=10
robot_image = pygame.transform.scale(robot_image, (robot_width, robot_length))
obstacle_static_image = pygame.transform.scale(obstacle_static_image, (cars_length*3, cars_length*3))
road_image = pygame.transform.scale(road_image, (40, WIDTH))
road_image = pygame.transform.rotate(road_image, 90)
obstacle_dynamic_image = pygame.transform.scale(obstacle_dynamic_image, (cars_width,cars_length))
obstacle_dynamic_image = pygame.transform.rotate(obstacle_dynamic_image, 90)
road_image2 = pygame.transform.scale(road_image2, (40, WIDTH))


class Robot:
    def __init__(self):
        self.width = robot_image.get_width()
        self.height = robot_image.get_height()

    def draw(self, surface, x, y, orientation):
        rotated_image = pygame.transform.rotate(robot_image, orientation - 90)

        # Dimentions of the rotated image to calculate center
        rotated_width, rotated_height = rotated_image.get_width(), rotated_image.get_height()

        # Centring the robot position
        blit_position = (x - rotated_width // 2, y + rotated_height // 2)
        surface.blit(rotated_image, blit_position)



# Dynamic Obstacles
class Obstacle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def draw(self, surface):
        surface.blit(obstacle_dynamic_image, (self.x-(cars_length//2), self.y+(cars_width//2)))


#Goal drawing
class SObstacle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def draw(self, surface):
        surface.blit(obstacle_static_image, (self.x-(self.width), self.y-(self.height//2)))




# Horizontal Road
class Road:
    def __init__(self, x_position, y_position):
        self.y_position = y_position
        self.x_position = x_position

    def draw(self, surface):
        surface.blit(road_image, (self.x_position, self.y_position+(cars_width//2)-5))

#Vertical Road
class Road2:
    def __init__(self, x_position, y_position):
        self.y_position = y_position
        self.x_position = x_position

    def draw(self, surface):
        surface.blit(road_image2, (self.x_position-25, self.y_position))


# Loads the robot data
def load_simulation(file_path):
    robot_states = []

    with open(file_path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            robot_state = {
                "x": float(row[0])*30+50-2100,  # x-pos
                "y": 700-float(row[1])*20-100,  # y-pos
                "orientation": math.degrees(float(row[2])),  # psi
                "velocity": float(row[3]),  # v
                "steering_angle": math.degrees(float(row[4])),  # delta
                #"acceleration": float(row[5])  # a
            }
            robot_states.append(robot_state)

    return robot_states



# Loads the positions of every obstacle
def load_obstacles_from_csv(directory_path):
    obstacles_per_frame = []
    files = [f for f in os.listdir(directory_path) if f.endswith('.csv')]
    files.sort()

    # Extract the positions from every file
    for file_name in files:
        file_path = os.path.join(directory_path, file_name)
        obstacle_positions = []

        with open(file_path, "r") as f:
            reader = csv.reader(f)

            for row in reader:
                x, y = float(row[0])*30+50-2100,700- float(row[1])*20-100
                obstacle_positions.append([x, y])

        obstacles_per_frame.append(obstacle_positions)

    return obstacles_per_frame


directory_path = "obstaculos_posicoes" 
obstacle_positions = load_obstacles_from_csv(directory_path)

robot = Robot()

robot_file = "robot_data.csv"  # Substituir pelo caminho correto
robot_states = load_simulation(robot_file)

robot_state = robot_states[len(robot_states)-1]
road2= Road2(robot_state["x"],robot_state["y"]) # Place the vertical road

# Looks at the positions of the obstacles to place the horizontal roads
n_roads=[]
for nobs in obstacle_positions:
            y=nobs[len(nobs)-1][1]
            if y not in n_roads:
                n_roads+=[y]

goal=SObstacle(robot_state["x"], robot_state["y"],26,26)



clock = pygame.time.Clock()  # Já inicializado
fps = 10  #set frames per secont
frame=244
print(robot_states[frame])
print(obstacle_positions[0][frame])

running = True
frame_index = 150
while running:
    screen.fill((255, 255, 255))  # Fundo branco
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Draws the roads
    road2.draw(screen)
    i=0
    while i<len(n_roads)-1:
        road = Road(0,n_roads[i])
        road.draw(screen)
        i+=1
    goal.draw(screen)

    # Draw the robot and obstacles for each frame
    if frame_index < len(robot_states):
        robot_state = robot_states[frame_index]
        
        # Obstacles
        for nobs in obstacle_positions:
            #if nobs[frame_index][0]!=-10.0:
            obs = Obstacle(nobs[frame_index][0], nobs[frame_index][1], 50, 50)
            obs.draw(screen)


        robot.draw(screen, robot_state["x"], robot_state["y"], robot_state["orientation"])

        # Shows informations
        font = pygame.font.SysFont(None, 24)
        info_text = f"Velocity: {robot_state['velocity']:.2f} | Delta: {robot_state['steering_angle']:.2f}"
        info_surface = font.render(info_text, True, (0, 0, 0))
        screen.blit(info_surface, (10, 10))

        frame_index += 1

    else:
        running = False  
    clock.tick(fps) # set the frames per second

    pygame.display.flip()
    clock.tick(30)

pygame.quit()
sys.exit()
