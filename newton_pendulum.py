import pygame as pg
import pymunk as pm
import numpy as np


'''
1. Constraints
    1.1 Solid rods
    1.2 Length
    1.3 Constr number
    1.4 Edge points coords

2. Balls
    2.1 Mass
    2.2 Moment
    2.3 Radius
    2.4 Initial coords
'''


class NewtonPendulum:
    def get_nearest_ball(self, x_coord: float, y_coord: float):
        #Method returns the nearest ball shape to the point with coords (x_coord, y_coord)    
        distances = [pm.Vec2d(x_coord, y_coord).get_distance(ball_shape.body.position) for ball_shape in self.__objects_shapes['balls_shapes']]
        return self.__objects_shapes['balls_shapes'][distances.index(min(distances))] 

    
    def __check_parameters(self, screen: pg.Surface, constr_num, constr_len, left_edge_point_coords, 
                           right_edge_point_coords, ball_mass, ball_moment, ball_radius):
        output_dict = {}
        try:
            if isinstance(constr_num, int) and constr_num >= 2:
                output_dict['constr_num'] = constr_num
            else:
                raise ValueError('constr_num error')
            
            if isinstance(constr_len, (int, float)) and constr_len > 0:
                output_dict['constr_len'] = constr_len
            else:
                raise ValueError('constr_len error') 

            
            statement1 = isinstance(left_edge_point_coords, (pm.Vec2d, tuple))
            statement2 = isinstance(right_edge_point_coords, (pm.Vec2d, tuple))
            statement3 = left_edge_point_coords[0] < right_edge_point_coords[0]
            statement4 = left_edge_point_coords[1] == right_edge_point_coords[1] and 0 < left_edge_point_coords[1] < screen.get_height()                               
            statement5 = left_edge_point_coords[0] > 0 and right_edge_point_coords[0] < screen.get_width()
            if all((statement1, statement2, statement3, statement4, statement5)):  
                output_dict['left_edge_point_coords'] = left_edge_point_coords  
                output_dict['right_edge_point_coords'] = right_edge_point_coords    
            else:
                raise ValueError('Fixed edge points error')
            
            if isinstance(ball_mass, (float, int)) and ball_mass > 0:
                output_dict['ball_mass'] = ball_mass
            else:
                raise ValueError('ball_mass error')
            
            if isinstance(ball_moment, (float, int)) and ball_moment > 0:
                output_dict['ball_moment'] = ball_moment
            else:
                raise ValueError('ball_moment error')
            
            dist = (right_edge_point_coords[0] - left_edge_point_coords[0]) / (2 * (constr_num - 1)) 
            if isinstance(ball_radius, (int, float)) and 0 < ball_radius <= dist:
                output_dict['ball_radius'] = ball_radius       
            else:
                raise ValueError('ball_radius error')
        
    
    
        except ValueError as ve:
            if str(ve) == 'constr_num error':
                print('constr_num parameter must be integer and >= 2')
            elif str(ve) == 'constr_len error':
                print('constr_len parameter must be int or float and > 0')
            elif str(ve) == 'Fixed edge points error':
            
                print('''Edges coords must have type tuple or pymunk.Vec2d, \n its 
                      x > 0 and y > 0 coords must have values < sizes of Pygame window and \n 
                      y coords of the left and right edges should be the same.''')
            elif str(ve) == 'ball_mass error':
                print('ball_mass parameter must be float or int and > 0')
            elif str(ve) == 'ball_moment error':
                print('ball_moment parameter must be float or int and > 0')
            else:
                print('''ball_radius parameter must be positive float or int and <= distance
                        between neighborhood fixed points / 2''')
            
        return output_dict
           
    
    
    def __init__(self, space: pm.Space, screen: pg.Surface, constr_num: int, constr_len: float, left_edge_point_coords, 
                 right_edge_point_coords, ball_mass: float, ball_moment: float, ball_radius: float):
        verified_parameters = self.__check_parameters(screen, constr_num, constr_len, left_edge_point_coords, right_edge_point_coords, 
                                               ball_mass, ball_moment, ball_radius)
        
        self.space, self.screen = space, screen
        self.constr_num, self.constr_len = verified_parameters['constr_num'], verified_parameters['constr_len']
        self.left_edge_point_coords = verified_parameters['left_edge_point_coords']
        self.right_edge_point_coords = verified_parameters['right_edge_point_coords']
        self.ball_mass, self.ball_moment = verified_parameters['ball_mass'], verified_parameters['ball_moment'] 
        self.ball_radius = verified_parameters['ball_radius'] 
        self.__objects_shapes = {'fixed_points_shapes': [], 'balls_shapes': [], 'rods': []}
        self.initial_coords = []

    
    def set_balls_initial_coords(self):
        for ball in self.__objects_shapes['balls_shapes']:
            self.initial_coords.append(ball.body.position)      

    
    def set_velocities(self, velocities_list = None, zero_velocities = True): 
        for i, ball in enumerate(self.__objects_shapes['balls_shapes']):
            ball.body.velocity =  pm.Vec2d(0, 0) if velocities_list is None and zero_velocities else velocities_list[i]

        
    def get_balls_initial_coords(self):
        return self.initial_coords
    
    
    def get_objects_shapes(self):
        return self.__objects_shapes
    

    def create_model(self, fixed_points_mass = 1.0, fixed_points_moment = 1.0, fixed_points_radius = 4):
        for x_coord in iter(np.linspace(self.left_edge_point_coords[0], self.right_edge_point_coords[0], self.constr_num)):
            fixed_point = pm.Body(fixed_points_mass, fixed_points_moment, body_type=pm.Body.STATIC)
            fixed_point.position = (x_coord, self.left_edge_point_coords[1])
            fixed_point_shape = pm.Circle(fixed_point, fixed_points_radius)
            ball = pm.Body(self.ball_mass, self.ball_moment, body_type = pm.Body.DYNAMIC)
            ball.position = (x_coord, self.left_edge_point_coords[1] + self.constr_len)
            ball_shape = pm.Circle(ball, self.ball_radius)
            ball_shape.elasticity, ball_shape.friction = 1, 1
            rod = pm.constraints.PinJoint(fixed_point, ball, (0,0), (0,0))
            self.space.add(fixed_point, fixed_point_shape, ball, ball_shape, rod)
            self.__objects_shapes['fixed_points_shapes'].append(fixed_point_shape)
            self.__objects_shapes['balls_shapes'].append(ball_shape)
            self.__objects_shapes['rods'].append(rod)

        self.set_balls_initial_coords()

        
    def draw(self, fixed_points_radius = 4, line_width = 3):
        for i in iter(range(self.constr_num)):
            pg.draw.circle(self.screen, (0,0,0), self.__objects_shapes['fixed_points_shapes'][i].body.position, fixed_points_radius) 
            pg.draw.line(self.screen, (0,0,0), self.__objects_shapes['rods'][i].a.position, self.__objects_shapes['rods'][i].b.position, line_width)
            pg.draw.circle(self.screen, (0,0,0), self.__objects_shapes['balls_shapes'][i].body.position, self.ball_radius)




if __name__ == '__main__':
    FPS = 60
    pg.init()
    screen = pg.display.set_mode((900, 600))
    clock = pg.time.Clock()
    space = pm.Space()
    space.gravity = (0,200)


    seven_joint_model = NewtonPendulum(space, screen, 7, 300, (250, 100), (650, 100), 40, 60, 33.3333333333)
    seven_joint_model.create_model()
    is_dragging, start_mouse_pos, start_body_pos = False, None, None

    while True:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()

            if event.type == pg.KEYDOWN:
                if event.key == pg.K_0:
                    #If you press a "0" key, this model will be restarted.
                    seven_joint_model.set_velocities()
                    for i, ball_shape in enumerate(seven_joint_model.get_objects_shapes()['balls_shapes']):
                        ball_shape.body.position = seven_joint_model.get_balls_initial_coords()[i]         
        
            if event.type == pg.MOUSEBUTTONDOWN:
                if event.button == 1:
                    '''
                mouse_x, mouse_y  = pg.mouse.get_pos()
                nearest_ball = seven_joint_model.get_nearest_ball(mouse_x, mouse_y)
                #space.bodies[len(space.bodies)-1].position = mouse_x, mouse_y 
                nearest_ball.body.position = mouse_x, mouse_y
                    '''
                    mouse_x, mouse_y  = pg.mouse.get_pos()
                    nearest_ball = seven_joint_model.get_nearest_ball(mouse_x, mouse_y)

                    if nearest_ball.point_query(event.pos):
                        is_dragging, start_mouse_pos, start_body_pos  = True, (mouse_x, mouse_y), nearest_ball.body.position
                    
            elif event.type == pg.MOUSEBUTTONUP:
                if event.button == 1:  
                    is_dragging, start_mouse_pos, start_body_pos = False, None, None
    
        if is_dragging:
            mouse_pos = pg.mouse.get_pos()
            displacement = mouse_pos[0] - start_mouse_pos[0], mouse_pos[1] - start_mouse_pos[1]
            nearest_ball.body.position = start_body_pos[0] + displacement[0], start_body_pos[1] + displacement[1]

        screen.fill((255,255,255))
        seven_joint_model.draw()
        space.step(1 / FPS)
        pg.display.update()
        clock.tick(FPS)

        















