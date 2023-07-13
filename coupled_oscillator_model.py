import pygame as pg
import pymunk as pm
import numpy as np


'''
Parameters of the model:
1. Charasteristics of fixed points:
    1.1 Coordinates of the fixed points
    1.2 Distance between fixed points (not necessary)
    1.3 Point mass, Point moment
    1.4 Radius, color
    

2. Characteristics of all constraints:
    2.1 Type of the vertical constraints (springs or solid rods)
    2.2 Type of the horizontal constraints (springs or solid rods)
    2.3 Position of horizontal constraints (center of vertical or free moving end)
    2.4 Number of the fixed points (number of vertical constraints)
    2.5 Number of horizontal constraints = 2.4 - 1
    2.6 Rest length, stiffness, damping of vertical constraints
    2.7 Rest length, stiffness, damping of horizontal constraints


3. Characteristics of all moving points:
    3.1 Its presence in the model
    3.2 Masses and moments of points
    3.3 Initial coordinates of the edge points
'''

class WindowSpaceInitializer:
    #Initialization of Pygame and Pymunk libraries, creating pygame window and pymunk space
    def __init__(self, wind_width: int, wind_length: int, gravity: float, fps: int, resizable = False):
        self.wind_width, self.wind_length = wind_width, wind_length
        self.gravity, self.fps = gravity if isinstance(gravity, (int, float, tuple)) else 100, fps 
        self.screen, self.clock, self.space = None, None, None 
        self.resizable = resizable
     

    def initialize(self):
        #Initialization of the window and space 
        pg.init() 
        self.screen = pg.display.set_mode((self.wind_width, self.wind_length), pg.RESIZABLE if self.resizable else 0)
        self.clock = pg.time.Clock()
        self.space = pm.Space()
        self.space.gravity = self.gravity if isinstance(self.gravity, tuple) else (0, self.gravity)


class CoupledOscillatorModel:
    '''
    Future updates:
        1. Add check_parameters method for checking all of user-setted parameters (it will be called in __init__())
        2. Add function for vertical attachment of the same exemplar of this model or another exemplar.
        3. Figure out behaviour of the model when user wants to change position of the point by mouse click.
            3.1 Maybe need to be changed method for calculating displacement of the point.
        4. Add method to GameLoop class for stop the moving     
        5. Add methdd to CoupledOscillatorModel class for changing distances between fixed points
            (make this distances configurable by the user) 
    '''
    '''
    def check_parameters(windowspaceinit: WindowSpaceInitializer, left_fixed_edge_coords, right_fixed_edge_coords): 
        try:
            st1 = isinstance(left_fixed_edge_coords, (tuple, pm.Vec2d)) and 0 < left_fixed_edge_coords[0] < right_fixed_edge_coords[0]
            st2 = isinstance(right_fixed_edge_coords, (tuple, pm.Vec2d)) and right_fixed_edge_coords[0] < windowspaceinit.wind_length
            st3 = 0 < left_fixed_edge_coords[1] < windowspaceinit.wind_width      
    '''
    
    
    def set_moving_point_parameters(self, moving_point_mass = 1, moving_point_moment = 1):
        self.__setattr__('moving_point_mass', moving_point_mass)
        self.__setattr__('moving_point_moment', moving_point_moment) 


    def set_styles(self, fp_color = (0,0,0), fp_radius = 4, mp_color = (0,0,0), mp_radius = 7, vert_line_width = 3):
        self.fixed_points_color, self.fixed_points_radius = fp_color, fp_radius
        self.mov_points_color, self.mov_points_radius = mp_color, mp_radius
        self.vert_line_width = vert_line_width
        self.hor_line_width = vert_line_width - 2 if self.vert_line_width > 2 else 1
    
    
    def set_constraint_parameters(self, constr_len, constr_stiffness, constr_damping, constr_type, constr_direction):
        if constr_type == 'spring':
            self.__setattr__('vert_stiffness' if constr_direction == 'vertical' else 'hor_stiffness', constr_stiffness)
            self.__setattr__('vert_damping' if constr_direction == 'vertical' else 'hor_damping', constr_damping) 
        self.__setattr__('vert_rest_len' if constr_direction == 'vertical' else 'hor_rest_len', constr_len)


    def __init__(self, windowspaceinit: WindowSpaceInitializer, left_fixed_edge_coords, right_fixed_edge_coords, 
                fixed_points_num = 2, vertical_constr_type = 'rod', horizontal_constr_type = 'spring', 
                vert_rest_len = 100, vert_stiffness = None, vert_damping = None, 
                hor_rest_len = None, hor_stiffness = None, hor_damping = None):
        self.space, self.screen = windowspaceinit.space,  windowspaceinit.screen
        self.left_fixed_edge_coords, self.right_fixed_edge_coords = left_fixed_edge_coords, right_fixed_edge_coords
        self.initial_points_coords = []
        self.fixed_points_num = fixed_points_num
        self.vertical_constr_type, self.horizontal_constr_type = vertical_constr_type, horizontal_constr_type
        self.set_constraint_parameters(vert_rest_len, vert_stiffness, vert_damping, 
                                       self.vertical_constr_type, 'vertical')
        self.set_constraint_parameters(hor_rest_len, hor_stiffness, hor_damping, 
                                       self.horizontal_constr_type, 'horizontal')
        self.set_moving_point_parameters()
        self.set_styles()
        self.__objects_shapes = {'fixed_points_shapes': [],'moving_points_shapes': [], 
                                 'vertical_constr': [], 'horizontal_constr': [],}

    
    def __add_object_to_space(self, *objects):
        for obj in objects:
            self.space.add(obj)
    
    
    def __create_fixed_point(self, x_coord, y_coord) -> pm.Body:
        fixed_point = pm.Body(body_type = pm.Body.STATIC)
        fixed_point.position = (x_coord, y_coord)
        fixed_point_shape = pm.Circle(fixed_point, self.fixed_points_radius)
        self.__objects_shapes['fixed_points_shapes'].append(fixed_point_shape)
        return fixed_point
    
     
    def set_moving_point_position(self, point: pm.Body, coords: tuple): point.position = coords

    
    def set_points_coords(self, coords_list: list, reversed_order = False):
        coords_list = coords_list if not reversed_order else coords_list[::-1]
        st1, st2 = len(coords_list) == self.fixed_points_num, len(coords_list) == 2 and self.fixed_points_num > 2
        if st1 or st2: coeff = 1 if st1 else -1 
        else: return "Error! Length of coords_list isn't equal number of constraints"
        for indx, coords in enumerate(coords_list):
            self.set_moving_point_position(self.__objects_shapes['moving_points_shapes'][coeff * indx].body, coords)    
        
        #for indx, coords in enumerate(coords_list):
            #if len(coords_list) == 2 and self.fixed_points_num > 2:
                #self.set_moving_point_position(self.__objects_shapes['moving_points_shapes'][-indx].body, coords)            
            #elif len(coords_list) == self.fixed_points_num:
                #self.set_moving_point_position(self.__objects_shapes['moving_points_shapes'][indx].body, coords)    
    

    def get_initial_point_coords(self): return self.initial_points_coords
        

    def __create_moving_point(self, x_coord, y_coord, moving_point_radius):
        moving_point = pm.Body(self.moving_point_mass, self.moving_point_moment, body_type = pm.Body.DYNAMIC)
        moving_point.position = (x_coord, y_coord)
        moving_point_shape = pm.Circle(moving_point, moving_point_radius)
        self.__objects_shapes['moving_points_shapes'].append(moving_point_shape)
        return moving_point

   
    def __create_constraint(self, left_end_body: pm.Body, right_end_body: pm.Body, constr_type: str, constr_direction: str):
        if constr_type == 'rod':
            constr = pm.constraints.PinJoint(left_end_body, right_end_body, (0,0), (0,0))
        else:
            rest_len, stiffness, damping = (self.vert_rest_len, self.vert_stiffness, self.vert_damping) if constr_direction == 'vertical' else (self.hor_rest_len, self.hor_stiffness, self.hor_damping)  
            constr = pm.constraints.DampedSpring(left_end_body, right_end_body, (0,0), (0,0), rest_len, stiffness, damping)
        self.__objects_shapes['vertical_constr'].append(constr) if constr_direction == 'vertical' else self.__objects_shapes['horizontal_constr'].append(constr)
        return constr
    

    def create_model(self):
        for x_coord in iter(np.linspace(self.left_fixed_edge_coords[0], self.right_fixed_edge_coords[0], self.fixed_points_num)):
            fixed_point = self.__create_fixed_point(x_coord, self.left_fixed_edge_coords[1])
            moving_point = self.__create_moving_point(x_coord, self.left_fixed_edge_coords[1] + self.vert_rest_len, self.mov_points_radius)
            self.initial_points_coords.append((x_coord, self.left_fixed_edge_coords[1] + self.vert_rest_len))
            vertical_constr = self.__create_constraint(fixed_point, moving_point, constr_type = self.vertical_constr_type, constr_direction = 'vertical')    
            self.__add_object_to_space(fixed_point, self.__objects_shapes['fixed_points_shapes'][-1], moving_point,
                              self.__objects_shapes['moving_points_shapes'][-1], vertical_constr) 
            
            if x_coord > self.left_fixed_edge_coords[0]:
                horizontal_constr = self.__create_constraint(self.__objects_shapes['moving_points_shapes'][-2].body, moving_point, 
                                                             self.horizontal_constr_type, constr_direction = 'horizontal')
                self.__add_object_to_space(horizontal_constr)
            
    
    def draw(self):
        vert_color = (0,0,0) if self.vertical_constr_type == 'rod' else (0,0,90)
        hor_color = (0,0,0) if self.horizontal_constr_type == 'rod' else (0,0,90)
        for i in iter(range(self.fixed_points_num)):
            pg.draw.circle(self.screen, self.fixed_points_color, self.__objects_shapes['fixed_points_shapes'][i].body.position, self.fixed_points_radius) 
            pg.draw.line(self.screen, vert_color, self.__objects_shapes['vertical_constr'][i].a.position, 
                        self.__objects_shapes['vertical_constr'][i].b.position, self.vert_line_width)
            pg.draw.circle(self.screen, self.mov_points_color, self.__objects_shapes['moving_points_shapes'][i].body.position, self.mov_points_radius)

        for hor_constr in iter(self.__objects_shapes['horizontal_constr']):
            pg.draw.line(self.screen, hor_color, hor_constr.a.position, hor_constr.b.position, self.hor_line_width)


    def get_clicked_point(self, mouse_pos: tuple) -> pm.Circle:
        check_dist = lambda d: d < 0  
        bool_list = [check_dist(obj_shape.point_query(mouse_pos)[2]) 
                     for obj_shape in self.__objects_shapes['moving_points_shapes']] 
        if all(map(lambda bool_input: not bool_input, bool_list)):
            return None
        return self.__objects_shapes['moving_points_shapes'][bool_list.index(True)]        
    

    def change_point_position(self, point: pm.Body, start_mouse_pos: tuple, cur_mouse_pos: tuple):
        displacement = cur_mouse_pos[0] - start_mouse_pos[0], cur_mouse_pos[1] - start_mouse_pos[1]
        new_coords = point.position[0] + displacement[0], point.position[1] + displacement[1]
        #point.force, point.torque = (0,0), 0
        self.set_moving_point_position(point, new_coords)  

    
    def restart_model(self):
        self.set_points_coords(self.get_initial_point_coords())
        for point_shape in iter(self.__objects_shapes['moving_points_shapes']): 
            point_shape.body.velocity = pm.Vec2d(0, 0)    
                


class GameLoop:
    def __init__(self, windowspaceinit: WindowSpaceInitializer, model):
        self.wsinit, self.model = windowspaceinit, model


    def play(self):
        running, is_dragging, nearest_point, start_mouse_pos = True, False, None, None
        while running:
            for event in pg.event.get():
                if event.type == pg.QUIT: pg.quit()
            
                if event.type == pg.MOUSEBUTTONDOWN and event.button == 1:
                    start_mouse_pos, is_dragging  = pg.mouse.get_pos(), True
                    nearest_point = self.model.get_clicked_point(start_mouse_pos)   
                    
                elif event.type == pg.MOUSEBUTTONUP and event.button == 1:
                    is_dragging, start_mouse_pos = False, None

                if event.type == pg.KEYDOWN and event.key == pg.K_0: self.model.restart_model()

            if is_dragging:
                if nearest_point is not None:
                    mouse_pos = pg.mouse.get_pos()
                    self.model.change_point_position(nearest_point.body, start_mouse_pos, mouse_pos)

            self.wsinit.screen.fill((255,255,255))
            self.model.draw()
            self.wsinit.space.step(1 / self.wsinit.fps)
            pg.display.update()
            wsinit.clock.tick(self.wsinit.fps)


            '''
            wsinit.screen.fill((255,255,255))
            couple_osc_model.draw()
            wsinit.space.step(1 / wsinit.fps)
            pg.display.update()
            wsinit.clock.tick(wsinit.fps)
            '''

wsinit = WindowSpaceInitializer(900, 600, 100, 60, True)
wsinit.initialize() 
couple_osc_model = CoupledOscillatorModel(wsinit, (100, 100), (700, 100), 5, hor_rest_len = 100, hor_damping = 5, hor_stiffness = 10, vert_rest_len = 200)
couple_osc_model.create_model()
gameloop = GameLoop(wsinit, couple_osc_model)
gameloop.play()





        
