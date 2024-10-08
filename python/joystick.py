import requests
import pygame

pygame.init()

#initialise the joystick module
pygame.joystick.init()

#define screen size
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 500

#create game window
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Joysticks")

#define font
font_size = 30
font = pygame.font.SysFont("Futura", font_size)

#function for outputting text onto the screen
def draw_text(text, font, text_col, x, y):
  img = font.render(text, True, text_col)
  screen.blit(img, (x, y))

#create clock for setting game frame rate
clock = pygame.time.Clock()
FPS = 60

#create empty list to store joysticks
joysticks = []

#create player rectangle
x = 350
y = 200
player = pygame.Rect(x, y, 100, 100)

#define player colour
col = "royalblue"

last_state = ""

commands = {
    "FORWARD": 'A',
    "STOP": 'F',
    "LEFT": 'E',
    "RIGHT": 'D',
    "BACK": 'S'
}


def send_to_raspi(state):
    # send an http post request with the state
    url = '172.22.43.111:5000'
    print("sending request")
    requests.get(f'http://{url}/execute/{ord(state)}')

    
def handle_movement(state):
    global last_state
    if last_state != state:
        last_state = state
        print(state)
        send_to_raspi(state)

run = True
while run:

    clock.tick(FPS)

  #update background
    screen.fill(pygame.Color("midnightblue"))

  #draw player
    player.topleft = (x, y)
    pygame.draw.rect(screen, pygame.Color(col), player)

  #show number of connected joysticks
    draw_text("Controllers: " + str(pygame.joystick.get_count()), font, pygame.Color("azure"), 10, 10)
    for joystick in joysticks:  
        draw_text("Battery Level: " + str(joystick.get_power_level()), font, pygame.Color("azure"), 10, 35)
        draw_text("Controller Type: " + str(joystick.get_name()), font, pygame.Color("azure"), 10, 60)
        draw_text("Number of axes: " + str(joystick.get_numaxes()), font, pygame.Color("azure"), 10, 85)

    for joystick in joysticks:
        vert_move = joystick.get_axis(1)
        yaw = joystick.get_axis(3)
        if vert_move < -0.8:
            handle_movement(commands["FORWARD"])
        elif vert_move > 0.8:
            handle_movement(commands["BACK"])
        elif yaw > 0.8:
            handle_movement(commands["RIGHT"])
        elif yaw < -0.8:
            handle_movement(commands["LEFT"])
        else:
            handle_movement(commands["STOP"])
            

  #event handler
    for event in pygame.event.get():
        if event.type == pygame.JOYDEVICEADDED:
            joy = pygame.joystick.Joystick(event.device_index)
            joysticks.append(joy)
        #quit program
        if event.type == pygame.QUIT:
            pass
            

  #update display
    pygame.display.flip()

pygame.quit()