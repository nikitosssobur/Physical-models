import pygame
import pymunk
'''
In this module you can watch the simulation of moving of the usual spring pendulum.
Try to change initial coodinates of the moving point.
Do your own experiments:)
'''


# Pygame initialization
pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()


# Pymunk initialization
space = pymunk.Space()
space.gravity = (0, 10)

# Creating fixed points
pivot = pymunk.Body(body_type=pymunk.Body.STATIC)
pivot.position = (400, 100)

point = pymunk.Body(1, 1, body_type=pymunk.Body.DYNAMIC)
#point.position = (400, 300)
point.position = (450, 100)


# Creating spring
spring = pymunk.DampedSpring(pivot, point, (0, 0), (0, 0), rest_length = 200, stiffness = 0.2, damping = 0.05)
#spring = pymunk.DampedRotarySpring(pivot, bob, 0, 5000, 50)

# Adding objects to the space
space.add(pivot, point, spring)

point.angular_velocity = 5


#check_click = False
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
        '''
        #Advanced idea: set initial coordinates of the moving point by the left button mouse click
        if event.type == pygame.MOUSEBUTTONDOWN and check_click == False:
            if event.button == 1:
                point = pymunk.Body(1, 1, body_type=pymunk.Body.DYNAMIC)
                point.position = event.pos
                point.angular_velocity = 5
                spring = pymunk.DampedSpring(pivot, point, (0, 0), (0, 0), rest_length = 200, stiffness = 0.2, damping = 0.05)
                space.add(point, spring)
                check_click = True
        '''     
    screen.fill((255, 255, 255))

    
    dt = 1.0 / 60.0
    for _ in range(10):
        space.step(dt)

    
    pygame.draw.line(screen, (0, 0, 0), pivot.position, point.position, 2)
    pygame.draw.circle(screen, (255, 0, 0), (int(pivot.position.x), int(pivot.position.y)), 5)
    pygame.draw.circle(screen, (0, 0, 255), (int(point.position.x), int(point.position.y)), 10)

    pygame.display.flip()
    
    clock.tick(60)
