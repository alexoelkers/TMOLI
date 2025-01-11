import pygame
import sys
import csv
import os
import math


# Inicializar o pygame
pygame.init()

# Configurações da janela
WIDTH, HEIGHT = 1100, 500
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Visualizador de Simulação 2D")
clock = pygame.time.Clock()

# Carregar imagens (substituir pelos caminhos reais das imagens)
robot_image = pygame.image.load("robot.png")
obstacle_static_image = pygame.image.load("armazem.png")
road_image = pygame.image.load("road.png")
obstacle_dynamic_image = pygame.image.load("Car.png")

# Redimensionar imagens para um tamanho padrão
robot_image = pygame.transform.scale(robot_image, (10, 10))
obstacle_static_image = pygame.transform.scale(obstacle_static_image, (25, 25))
road_image = pygame.transform.scale(road_image, (150, HEIGHT))
obstacle_dynamic_image = pygame.transform.scale(obstacle_dynamic_image, (10, 10))
obstacle_dynamic_image = pygame.transform.rotate(obstacle_dynamic_image, 90)


# Classe para desenhar o Robô
class Robot:
    def __init__(self):
        self.width = robot_image.get_width()
        self.height = robot_image.get_height()

    def draw(self, surface, x, y, orientation):
        rotated_image = pygame.transform.rotate(robot_image, -orientation-90)
        surface.blit(rotated_image, (x - self.width // 2, y - self.height // 2))


# Classe para Obstáculos
class Obstacle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def draw(self, surface):
        surface.blit(obstacle_dynamic_image, (self.x - self.width // 2, self.y - self.height // 2))


# Classe para desenhar a Estrada
class Road:
    def __init__(self, x_position, y_position):
        self.y_position = y_position
        self.x_position = x_position

    def draw(self, surface):
        surface.blit(road_image, (self.x_position, self.y_position))


# Carregar a simulação de um arquivo CSV (dados do robô)
def load_simulation(file_path):
    robot_states = []

    with open(file_path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            robot_state = {
                "x": float(row[0])*10+50,  # x-pos
                "y": float(row[1])*10+50,  # y-pos
                "orientation": math.degrees(float(row[2])),  # psi
                "velocity": float(row[3]),  # v
                "steering_angle": math.degrees(float(row[4])),  # delta
                "acceleration": float(row[5])  # a
            }
            robot_states.append(robot_state)

    return robot_states

'''
# Carregar obstáculos de um arquivo CSV
def load_obstacles(file_path):
    obstacles_per_frame = []

    with open(file_path, "r") as f:
        for line in f:
            # Dividir a linha por espaços e converter os números para float
            obstacle_positions = [list(map(float, obs.split())) for obs in line.strip().split('],[')]
            obstacles_per_frame.append(obstacle_positions)

    return obstacles_per_frame
'''


# Carregar obstáculos a partir de ficheiros CSV diferentes
def load_obstacles_from_csv(directory_path):
    obstacles_per_frame = []

    # Listar todos os ficheiros CSV na pasta
    files = [f for f in os.listdir(directory_path) if f.endswith('.csv')]

    # Ordenar os ficheiros (se necessário, baseado no nome ou outra lógica)
    files.sort()

    # Carregar as posições de cada obstáculo
    for file_name in files:
        file_path = os.path.join(directory_path, file_name)
        
        # Lista para armazenar as posições de um único obstáculo ao longo do tempo
        obstacle_positions = []

        with open(file_path, "r") as f:
            reader = csv.reader(f)
            next(reader)  # Pular a linha de cabeçalho

            for row in reader:
                # As posições estão nas duas primeiras colunas
                x, y = float(row[0])*10+50, float(row[1])*10+50
                obstacle_positions.append([x, y])

        obstacles_per_frame.append(obstacle_positions)

    return obstacles_per_frame

# Exemplo de uso:
directory_path = "obstaculos_posicoes"  # Pasta onde os ficheiros CSV foram salvos
obstacle_positions = load_obstacles_from_csv(directory_path)
print(obstacle_positions)


# Inicializar classes de desenho
robot = Robot()
road = Road(WIDTH // 2 - 75, 0)  # Centralizar a estrada

# Carregar os dados da simulação
robot_file = "robot_data.csv"  # Substituir pelo caminho correto
robot_states = load_simulation(robot_file)

# Carregar os obstáculos
#obstacle_file = "obstacles_data.csv"  # Substituir pelo caminho correto
#obstacle_positions = load_obstacles(obstacle_file)

# Loop principal de visualização
running = True
frame_index = 0
while running:
    screen.fill((255, 255, 255))  # Fundo branco

    # Eventos
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Desenhar a estrada
    road.draw(screen)

    # Obter o estado atual do robô e obstáculos
    if frame_index < len(robot_states):
        robot_state = robot_states[frame_index]
        #current_obstacles = obstacle_positions[frame_index]  # Posições dos obstáculos para o frame atual

        # Desenhar o robô
        robot.draw(screen, robot_state["x"], robot_state["y"], robot_state["orientation"])

        # Desenhar os obstáculos
        for nobs in obstacle_positions:
            if nobs[frame_index][0]!=-10.0:
                obs = Obstacle(nobs[frame_index][0], nobs[frame_index][1], 50, 50)
                obs.draw(screen)
            #if nobs != [-10.0, -10.0]:  # Ignorar posições fora do mapa
                #obs = Obstacle(obs_pos[0], obs_pos[1], 50, 50)  # Criar o obstáculo (exemplo com tamanho fixo)
                #obs.draw(screen)  # Desenhar o obstáculo

        # Exibir outras informações (opcional)
        font = pygame.font.SysFont(None, 24)
        info_text = f"Velocity: {robot_state['velocity']:.2f} | Delta: {robot_state['steering_angle']:.2f} | Acceleration: {robot_state['acceleration']:.2f}"
        info_surface = font.render(info_text, True, (0, 0, 0))
        screen.blit(info_surface, (10, 10))

        # Avançar para o próximo frame
        frame_index += 1
        print(frame_index)
    else:
        running = False  # Encerrar o loop quando a simulação terminar

    # Atualizar a tela
    pygame.display.flip()
    clock.tick(30)

# Sair do pygame
pygame.quit()
sys.exit()