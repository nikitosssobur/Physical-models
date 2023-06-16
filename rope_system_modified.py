import numpy as np
import pymunk as pm
import pygame as pg
import pymunk.pygame_util
import datetime
import time 


FPS = 60
pg.init()
screen = pg.display.set_mode((800, 650))
clock = pg.time.Clock()
space = pm.Space()
space.gravity = (0,100)
do_snapshots = False  #Do (True) or not (False) snapshots 
interval = 1.05   #Interval between snapshots in seconds    
num_snapshots = 12  #Number of shots
snapshot_delay = 2.2 #Delay in taking shots in seconds 

#Hyperparameters
base_points_number = 20 #12 #24 #18            #the number of base points without a central point 
loaded_rope_points_number = 3     #the number of loaded_rope points without a central point and without a load point
x_left_coord, x_right_coord = 30, 770  #coordinates of fixed points along the x axis 
y_coord = 120                          #y coordinate of fixed points and base rope points 
x_center_point_coord =  (770 + 30) / 2  #coordinate of the central point of the base         
center_weight_distance = 150            #distance from the central point of the base to the load point without deformation
base_rope_mass = 8                      #mass of the entire rope
base_rope_stiffness =  7  #3.8 #7       #stiffness coefficient of the entire base rope
#base_spring_stiffness, base_spring_damping = 1.5, 0.05#1.5, 0.0015 #1.5, 0.02   #stiffness and coefficient of viscosity for the springs of the base
base_spring_stiffness, base_spring_damping = base_rope_stiffness * base_points_number, 0.05 
loaded_rope_stiffness, loaded_rope_damping = base_spring_stiffness / 2, base_spring_damping / 2 
delta_len = 150                          #the size of the extension of the base springs at the initial moment of time
#base_point_mass, base_point_moment = 0.001, 1    #mass and moment of inertia of the base points
base_point_mass, base_point_moment = base_rope_mass / base_points_number, 1
static_edges_radius = 10                         #static points radius
loaded_rope_point_mass, loaded_rope_point_moment = 1, 1      #mass and moment of inertia of the loaded rope points
weight_point_mass, weight_point_moment = 30, 5               #mass and moment of inertia of the weight point
static_point_mass, static_point_moment = 1, 0                
moving_points_radius = 7                                     #moving points radius
x_bias, y_bias = 0, 0                 #displacement along the x-axis and along the y-axis of the load point relative to the central point


def create_static_edges(space: pm.Space) -> dict:
    #Creating fixed edge points
    left_edge_body = pm.Body(static_point_mass, static_point_moment, body_type=pm.Body.STATIC)
    right_edge_body = pm.Body(static_point_mass, static_point_moment, body_type=pm.Body.STATIC)
    left_edge_body.position, right_edge_body.position = (x_left_coord, y_coord), (x_right_coord, y_coord)
    left_shape, right_shape = pm.Circle(left_edge_body, static_edges_radius), pm.Circle(right_edge_body, static_edges_radius)
    space.add(left_edge_body, right_edge_body, left_shape, right_shape)
    return {'bodies': (left_edge_body, right_edge_body), 'shapes': (left_shape, right_shape)}


edges = create_static_edges(space)


def draw_edges(edges: dict) -> None:
    #Function draws edges 
    
    for edge in edges['shapes']:
        pos_tuple = (int(edge.body.position.x),int(edge.body.position.y))
        pg.draw.circle(screen, (0, 0, 50), pos_tuple, static_edges_radius)


def add_moving_circles(space: pm.Space, edges: dict) -> dict:
    #Function add center moving points 
    center_point = pm.Body(base_point_mass, base_point_moment, body_type = pm.Body.DYNAMIC)
    center_point.position = pm.Vec2d(x_center_point_coord, y_coord)
    shape = pm.Circle(center_point, moving_points_radius)

    weight_point = pm.Body(weight_point_mass, weight_point_moment, body_type = pm.Body.DYNAMIC)
    weight_point.position = pm.Vec2d(x_center_point_coord + x_bias, y_coord - center_weight_distance - y_bias)
    weight_point_shape = pm.Circle(weight_point, moving_points_radius)
    center_point.velocity, weight_point.velocity = pm.Vec2d(0, 0), pymunk.Vec2d(0, 0)

    point_shapes, point_bodies, springs = [],[],[]
    x_base_point_coords = np.linspace(edges['bodies'][0].position[0], edges['bodies'][1].position[0], base_points_number + 3) #[1:]
    y_loaded_rope_coords = np.linspace(center_point.position.y, weight_point.position.y, loaded_rope_points_number + 2)[1:]
    
    #stretched_len_base_spring = abs(edges['bodies'][0].position[0] - x_base_point_coords[1])
    #rest_len_loaded_rope_spring = abs(y_coord - y_loaded_rope_coords[1])
    stretched_len_base_spring = edges['bodies'][0].position.get_distance((x_base_point_coords[1], y_coord))
    delta = delta_len if 0 <= delta_len < stretched_len_base_spring else 0.5 * stretched_len_base_spring 
    rest_len_base_spring = stretched_len_base_spring -  delta
    rest_len_loaded_rope_spring = center_point.position.get_distance((x_center_point_coord, y_loaded_rope_coords[0]))
    

    for i in range(len(x_base_point_coords)):
        #loop for adding points of the base spring
        if i == 0:
            continue
        elif i == len(x_base_point_coords) - 1:
            spring = pm.constraints.DampedSpring(point_bodies[-1], edges['bodies'][1], anchor_a = (0,0), anchor_b = (0,0),
                                          rest_length = rest_len_base_spring, stiffness = base_spring_stiffness,
                                          damping = base_spring_damping)
            
        else:
            left_point = edges['bodies'][0] if i == 1 else point_bodies[-1]
            
            if i == int(len(x_base_point_coords) / 2):   #if i == index of the center point x_coord
                point = center_point
                point_shape = shape
                
            else:
                point = pm.Body(base_point_mass, base_point_moment, body_type = pm.Body.DYNAMIC)
                point.position = pm.Vec2d(x_base_point_coords[i], y_coord)
                point_shape = pm.Circle(point, moving_points_radius)
                
            
            spring = pm.constraints.DampedSpring(left_point, point, anchor_a = (0,0), anchor_b = (0,0),
                                          rest_length = rest_len_base_spring, stiffness = base_spring_stiffness,
                                          damping = base_spring_damping)
            
            
            point.velocity = pm.Vec2d(0, 0)
            point_shapes.append(point_shape)
            point_bodies.append(point)
            space.add(point, point_shape)

         
        spring.activate_bodies()
        springs.append(spring) 
        space.add(spring)
    
   
    loaded_rope_point_shapes, loaded_rope_point_bodies, loaded_rope_point_springs = [],[],[]
    for i in range(len(y_loaded_rope_coords)):
        #Loop for adding points of loaded spring
        if i == len(y_loaded_rope_coords) - 1:
            a_point = center_point if i == 0 else loaded_rope_point_bodies[i - 1]
            point, point_shape  = weight_point, weight_point_shape
            
        else:
            a_point = center_point if i == 0 else loaded_rope_point_bodies[i - 1]
            point = pm.Body(loaded_rope_point_mass, loaded_rope_point_moment, body_type = pm.Body.DYNAMIC)
            point.position = pm.Vec2d(x_center_point_coord, y_loaded_rope_coords[i])
            point_shape = pm.Circle(point, moving_points_radius)

        spring = pm.constraints.DampedSpring(a_point, point, anchor_a = (0,0), anchor_b = (0,0),
                                          rest_length = rest_len_loaded_rope_spring, 
                                          stiffness = loaded_rope_stiffness,
                                          damping = loaded_rope_damping)


        loaded_rope_point_shapes.append(point_shape)
        loaded_rope_point_bodies.append(point)
        space.add(point, point_shape)

        loaded_rope_point_springs.append(spring) 
        spring.activate_bodies()
        space.add(spring)
        
         
    return {'base_shapes': point_shapes, 'base_springs': springs, 'loaded_rope_shapes': loaded_rope_point_shapes,
            'loaded_rope_springs': loaded_rope_point_springs}
           
       
    

def draw_circles(circles: dict) -> None:
    #Drawing all objects using pygame interface
    for base_shape in circles['base_shapes']:
        pos_tuple = (base_shape.body.position.x, base_shape.body.position.y)
        pg.draw.circle(screen, (0,0,50), pos_tuple, moving_points_radius)


    for i, lr_shape in enumerate(circles['loaded_rope_shapes']):
        pos_tuple = (lr_shape.body.position.x, lr_shape.body.position.y)
        colors = (0, 0, 250) if i == len(circles['loaded_rope_shapes']) - 1 else (0,0,50)
        pg.draw.circle(screen, colors, pos_tuple, moving_points_radius)
        
     
    for base_spring in circles['base_springs']:
        pg.draw.line(screen, (0,0,0), (base_spring.a.position.x, base_spring.a.position.y),
                   (base_spring.b.position.x, base_spring.b.position.y), 1)

    
    for lr_spring in circles['loaded_rope_springs']:
        pg.draw.line(screen, (0,0,0), (lr_spring.a.position.x, lr_spring.a.position.y),
                     (lr_spring.b.position.x, lr_spring.b.position.y),  1)
        
    

circles = add_moving_circles(space, edges)

snapshot_timer = 0
snapshot_counter = 0

loop_start_time = time.time()
while True:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            pg.quit()
            
    screen.fill((255,255,255))
    draw_edges(edges)
    draw_circles(circles)
    space.step(1/FPS)
    if do_snapshots and time.time() > loop_start_time + snapshot_delay: 
        #This block of code doing snapshots.
        snapshot_timer += clock.get_time()
        if snapshot_timer >= interval * 1000 and snapshot_counter < num_snapshots:
            snapshot_datetime = datetime.datetime.now().strftime("%d%m%Y_%H%M%S") 
            pg.image.save(screen, f"screenshot_{snapshot_counter}_{snapshot_datetime}.png")
            snapshot_counter += 1
            snapshot_timer = 0
    pg.display.update()
    clock.tick(FPS)
    
