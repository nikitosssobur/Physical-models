import pymunk as pm
import pygame as pg
import pymunk.pygame_util


'''
In this python program there are 2 fixed points by the edges, central moving point, which is joined to
fixed points by 2 springs (spring1, spring2) and to loaded point by spring3.
You can change values of hyperparameters and see changes of behaviour of this system.
'''

FPS = 60
pg.init()
screen = pg.display.set_mode((800, 650))
clock = pg.time.Clock()
space = pm.Space()
space.gravity = (0,100)


#Hyperparameters
x_left_coord, x_right_coord = 30, 770
y_coord = 180
x_center_point_coord =  (770 + 30) / 2
center_weight_distance = 150
base_spring_stiffness, base_spring_damping = 5.5, 0.05
delta_len = 0.05
base_point_mass, base_point_moment = 0, 0
static_edges_radius = 10
weight_point_mass, weight_point_moment = 50, 10
center_point_mass, center_point_moment =  0.01, 1 #100, 3 
static_point_mass, static_point_moment = 1, 0
moving_points_radius = 7


def create_static_edges(space):
    #Creating 2 fixed points by the edges
    left_edge_body = pm.Body(static_point_mass, static_point_moment, body_type=pm.Body.STATIC)
    right_edge_body = pm.Body(static_point_mass, static_point_moment, body_type=pm.Body.STATIC)
    left_edge_body.position, right_edge_body.position = (x_left_coord, y_coord), (x_right_coord, y_coord)
    left_shape, right_shape = pm.Circle(left_edge_body, static_edges_radius), pm.Circle(right_edge_body, static_edges_radius)
    space.add(left_edge_body, right_edge_body, left_shape, right_shape)
    return {'bodies': (left_edge_body, right_edge_body), 'shapes': (left_shape, right_shape)}

edges = create_static_edges(space)


def draw_edges(edges):
    #Drawing fixed edges on the screen
    for edge in edges['shapes']:
        pos_tuple = edge.body.position.x, edge.body.position.y
        pg.draw.circle(screen, (0, 0, 50), pos_tuple, static_edges_radius)


def add_moving_circles(space, edges):
    #Adding central moving points.  
    center_point = pm.Body(center_point_mass, center_point_moment, body_type=pm.Body.DYNAMIC)       
    center_point.position = pm.Vec2d(x_center_point_coord , y_coord)
    shape = pm.Circle(center_point, moving_points_radius)
    
    weight_point = pm.Body(weight_point_mass, weight_point_moment, body_type=pm.Body.DYNAMIC)       
    weight_point.position = pm.Vec2d(x_center_point_coord + 5, y_coord - center_weight_distance)
    weight_point_shape = pm.Circle(weight_point, moving_points_radius)
    
    stretched_len_base_spring = center_point.position.get_distance((edges['bodies'][0].position))
    rest_len_base_spring = stretched_len_base_spring - delta_len 
    
    spring1 = pm.constraints.DampedSpring(edges['bodies'][0], center_point,       #left spring
                                          anchor_a = (0,0),
                                          anchor_b = (0,0),
                                          rest_length = rest_len_base_spring, 
                                          stiffness = base_spring_stiffness,
                                          damping = base_spring_damping)
    spring2 = pm.constraints.DampedSpring(center_point, edges['bodies'][1],       #right spring
                                          anchor_a = (0,0),
                                          anchor_b = (0,0),
                                          rest_length = rest_len_base_spring, 
                                          stiffness = base_spring_stiffness,
                                          damping = base_spring_damping)
    spring3 = pm.constraints.DampedSpring(center_point, weight_point,             #loaded spring
                                          anchor_a = (0,0),
                                          anchor_b = (0,0),
                                          rest_length = center_weight_distance, 
                                          stiffness = base_spring_stiffness / 2,
                                          damping = base_spring_damping / 2)
    
    spring1.activate_bodies()       
    spring2.activate_bodies()
    spring3.activate_bodies() 
    space.add(center_point, shape, weight_point, weight_point_shape, spring1, spring2, spring3)
    return (shape, weight_point_shape), spring1, spring2, spring3    
    

def draw_circles(circles):
    #This function draws all moving points and all springs between them.
    
    center_pos_tuple = (circles[0][0].body.position.x, circles[0][0].body.position.y)
    pg.draw.circle(screen, (0,0,50), center_pos_tuple, moving_points_radius)
    weight_pos_tuple = (circles[0][1].body.position.x, circles[0][1].body.position.y)
    pg.draw.circle(screen, (0,0,50), weight_pos_tuple, moving_points_radius)
    
    #drawing springs
    pg.draw.line(screen, (0,0,0), circles[1].a.position, center_pos_tuple, 1)
    pg.draw.line(screen, (0,0,0), center_pos_tuple, circles[2].b.position, 1) 
    pg.draw.line(screen, (0,0,0), center_pos_tuple, weight_pos_tuple, 1) 




circles = add_moving_circles(space, edges)
while True:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            pg.quit()
            
    screen.fill((255,255,255))
    draw_edges(edges)
    draw_circles(circles)
    space.step(1/FPS)
    pg.display.update()
    clock.tick(FPS)


