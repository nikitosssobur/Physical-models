import pygame
import pymunk
'''
This model consists of two fixed points and falling center point attached to them by two springs.   
'''

# Pygame initialization
pygame.init()
screen = pygame.display.set_mode((800, 600))
clock = pygame.time.Clock()

# Pymunk initialization
space = pymunk.Space()
space.gravity = (0, 10)

# Creating fixed points
left_pivot = pymunk.Body(body_type=pymunk.Body.STATIC)
left_pivot.position = (200, 100)
right_pivot = pymunk.Body(body_type=pymunk.Body.STATIC)
right_pivot.position = (600, 100)


#Creating moving loaded point body. 
loaded_point = pymunk.Body(7, 1, body_type=pymunk.Body.DYNAMIC)
loaded_point.position = (400, 100)


# Creating springs
spring1 = pymunk.DampedSpring(left_pivot, loaded_point, (0, 0), (0, 0), rest_length = 100, stiffness = 0.2, damping = 0.05)
spring2 = pymunk.DampedSpring(loaded_point, right_pivot, (0, 0), (0, 0), rest_length = 100, stiffness = 0.2, damping = 0.05)

# Adding all of created objects to the space
space.add(left_pivot, right_pivot, loaded_point, spring1, spring2)

#Initial velocity vector for center point 
loaded_point.velocity = pymunk.Vec2d(0, 0)

# Main game loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()

    screen.fill((255, 255, 255))

    # The step of Pymunk simulation
    dt = 1.0 / 60.0
    for _ in range(10):
        space.step(dt)

    # Draw springs and points
    pygame.draw.line(screen, (0, 0, 0), left_pivot.position, loaded_point.position, 2)
    pygame.draw.line(screen, (0, 0, 0), loaded_point.position, right_pivot.position, 2) 
    pygame.draw.circle(screen, (255, 0, 0), (int(left_pivot.position.x), int(left_pivot.position.y)), 5)
    pygame.draw.circle(screen, (255, 0, 0), (int(right_pivot.position.x), int(right_pivot.position.y)), 5) 
    pygame.draw.circle(screen, (0, 0, 255), (int(loaded_point.position.x), int(loaded_point.position.y)), 10)

    # Screen update
    pygame.display.flip()
    clock.tick(60)
