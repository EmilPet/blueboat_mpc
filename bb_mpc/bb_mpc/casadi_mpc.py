# Based on code by Mohamed W. Mehrez, "MPC and MHE implementation in Matlab using CasADi" (2019)
#!/usr/bin/env python3

import rclpy
import pyproj
import numpy as np
import math
import time
from casadi import *
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import WaypointSetCurrent
from mavros_msgs.srv import WaypointPull    
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import WaypointList
from sensor_msgs.msg import NavSatFix
from functools import partial
import matplotlib.pyplot as plt


class MPCNode(Node):
   

    def __init__(self):
        # Initialising node
        super().__init__("casadi_mpc")

        self.current_ref = 0        

        ### INITIALIZING MPC ###
        self.h = 0.1
        h = self.h
        self.N = 30
        N=self.N
        
        self.x_ref_ned = []
        self.y_ref_ned = []
        self.equations = []
        #x pos: -46.75629806503105 , y pos: 52.32529830852493
        obstacle_x = -46.75629806503105
        obstacle_y = 52.32529830852493
        obstacle_diameter = 5
        vehicle_diameter = 5

        v_max = 4
        v_min = 0
 
        omega_max = pi/4
        omega_min = -omega_max

        self.mpc_iter = 0

        x = SX.sym('x')
        y = SX.sym('y')
        theta = SX.sym('theta')

        states = vertcat(x,y,theta)
        n_states = 3

        v = SX.sym('v')
        omega = SX.sym('omega')

        controls = vertcat(v,omega)
        n_controls = 2

        states_der = vertcat(v*cos(theta),v*sin(theta),omega)

        f = Function('f',[states,controls],[states_der])
        self.f = f
        U = SX.sym('U',n_controls, N)
        P = SX.sym('P',n_states + N*n_states)

        X = SX.sym('X', n_states,(N+1))

        obj = 0
        g = []

        Q = np.diag([15, 15, 0.1])  # weighing matrices (states)
        R = np.diag([5, 2])  # weighing matrices (controls)

        st = X[:,0]
        g.extend([st-P[0:3]])

        for k in range(N):
            st = X[:,k]
            con= U[:,k]
            obj = mtimes([(st - P[k*3+3:k*3 + 6]).T, Q, (st - P[k*3+3:k*3 + 6])]) + mtimes([con.T, R, con])
            #print(P[k*3+3:k*3 + 6])
            st_next = X[:,k+1]
            k1 = f(st, con)
            k2 = f(st + h / 2 * k1, con)
            k3 = f(st + h / 2 * k2, con)
            k4 = f(st + h * k3, con)
            st_next_RK4 = st + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            
            g.extend([st_next - st_next_RK4])
            
        #x pos: -28.747343498268965 , y pos: 27.957651477337425

        for k in range(N+1):
            g.extend([-sqrt((X[0,k] - obstacle_x)**2 + (X[1,k]- obstacle_y)**2) + (vehicle_diameter/2 + obstacle_diameter/2)])
       
        # make the decision variable one column vector
        OPT_variables = vertcat(reshape(X, 3 * (N + 1), 1), reshape(U, 2 * N, 1))

        nlp_prob = {'f': obj, 'x': OPT_variables, 'g': vertcat(*g), 'p': P}

        opts = {'ipopt.max_iter': 2000,
                'ipopt.print_level': 0,
                'print_time': 0,
                'ipopt.acceptable_tol': 1e-8,
                'ipopt.acceptable_obj_change_tol': 1e-6}

        self.solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

        self.args = {}

        # self.args['lbg'] =  # Collav lower bounds
        # self.args['ubg'] =  # Collav upper bounds

        self.args['lbg'] = [0] * (3 * (N + 1)) #+ [-inf] * (N+1) # Equality constraints
        self.args['ubg'] = [0] * (3 * (N + 1)) #+ [0] * (N+1) # Equality constraints


        self.args['lbg'] += [-inf] * (N+1) # Equality constraints
        self.args['ubg'] += [0] * (N+1) # Equality constraints


        self.args['lbx'] = [-inf] * 3 * (N + 1) + [v_min, omega_min] * N
        self.args['ubx'] = [inf] * 3 * (N + 1) + [v_max, omega_max] * N
        self.args['p'] = np.zeros(n_states + N * n_states) 

        self.x0 = [0,0,0]
        self.xs = [0,0,0]
        self.u0 = np.zeros((self.N, 2))  # two control inputs for each robot
        self.X0 = np.tile(self.x0, (self.N + 1, 1))  # initialization of the states decision variables
        
        # Setting quality of service profile
        qos_profile = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT
        )

        ### PUBLISHERS ###
        self.cmd_vel_pub_ = self.create_publisher(
            PositionTarget, 
            "/mavros/setpoint_raw/local", 
            10
            )
        
        self.reference_pos_pub_ = self.create_publisher(
            NavSatFix,
            "/current_waypoint/global_position",
            10
        )
        
        ### SUBSCRIBERS ###
        self.velocity_sub_ = self.create_subscription(
            TwistStamped,
            "/mavros/local_position/velocity_local",
            self.velocity_callback,
            qos_profile
        )
        
        self.pos_local_sub_ = self.create_subscription(
            PoseStamped, 
            "/mavros/local_position/pose", 
            self.pos_local_callback,
            qos_profile
        )
        
        self.waypoint_sub_ = self.create_subscription(
            WaypointList,
            "/mavros/mission/waypoints",
            self.waypoint_callback,
            qos_profile
        )

        ### Running MPC and publishing controls###
        
        self.run_mpc_timer = self.create_timer(0.1,self.run_mpc)

        # Informing node startup
        self.get_logger().info("Starting mpc node")





    ### SERVICES ###

    # Set waypoint service - sets the current waypoint to wp_seq
    def call_set_waypoint_service(self, wp_seq):
        client = self.create_client(WaypointSetCurrent, "/mavros/mission/set_current")
        
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        
        request = WaypointSetCurrent.Request()
        request.wp_seq = wp_seq

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_waypoint))

    def callback_set_waypoint(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))    


    # Set mode service - sets the vehicles flight mode      
    def call_set_mode_service(self, flight_mode):
        client = self.create_client(SetMode, "/mavros/set_mode")
        
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")
        
        request = SetMode.Request()
        request.custom_mode = flight_mode

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_waypoint))

    def callback_set_mode(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))


    ### SUBSCRIBER CALLBACKS ###

    # Velocity callback - calculates the angle of the velocity vector
    def velocity_callback(self,msg:TwistStamped):
        vy = msg.twist.linear.y
        vx = msg.twist.linear.x

        self.velocity_angle_rad = math.atan2(vy, vx)  


    # Waypoint callback - extracts waypoints and converts position to local NED frame
    # NB! Doesn't work atm
    def waypoint_callback(self, msg:WaypointList):
        waypoints = msg.waypoints
        #self.get_logger().info(str(waypoints))
        x_ref_wgs84 = []
        y_ref_wgs84 = []

        ###IDE###
        #Transformer globale koordinater til UTM (i riktig sone)
        #Sett første globale koordinater til nullpkt med ned orientering  
        WGS84 = 'epsg:4326'   # WGS84
        UTM = 'epsg:28355'  # UTM zone 55S  
        latlon_utm_transformer = pyproj.Transformer.from_crs(WGS84, UTM)

        for i, waypoint in enumerate(waypoints):
            if waypoint.x_lat != 0:
                global_lat = waypoint.x_lat
                global_lon = waypoint.y_long

                x_ref_wgs84.append(global_lat)
                y_ref_wgs84.append(global_lon)

        x_ref_wgs84 = [float(data) for data in x_ref_wgs84]
        y_ref_wgs84 = [float(data) for data in y_ref_wgs84]
        
        x_0 , y_0 = latlon_utm_transformer.transform(x_ref_wgs84[0],y_ref_wgs84[0])

        for i in range(len(x_ref_wgs84)):
            x_utm, y_utm = latlon_utm_transformer.transform(x_ref_wgs84[i],y_ref_wgs84[i])

            self.x_ref_ned.append(x_utm-x_0)
            self.y_ref_ned.append(y_utm-y_0)

        for i in range(len(self.x_ref_ned)-1):

            if self.x_ref_ned[i+1]-self.x_ref_ned[i]== 0:
                slope = 0
                intercept = self.x_ref_ned[i]
            else:
                slope = (self.y_ref_ned[i+1]-self.y_ref_ned[i])/(self.x_ref_ned[i+1]-self.x_ref_ned[i])
                intercept = self.y_ref_ned[i] - slope*self.x_ref_ned[i]
            self.equations.append((slope,intercept))

        self.current_ref = msg.current_seq
        
        cmd = NavSatFix()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.stamp
        cmd.latitude = x_ref_wgs84[self.current_ref - 1]
        cmd.longitude = y_ref_wgs84[self.current_ref - 1]
        self.reference_pos_pub_.publish(cmd)



    def pos_local_callback(self,msg:PoseStamped):
        
        # Processing local position
        East = msg.pose.position.x
        North = msg.pose.position.y
        Theta = self.velocity_angle_rad

        if len(self.x_ref_ned) != 0:
            East_ref = self.x_ref_ned[self.current_ref]
            North_ref = self.y_ref_ned[self.current_ref]
        else: 
            East_ref = 100
            North_ref = 100

        t0 = 0
        self.x0 = [East,North,Theta]
        self.xs = [East_ref,North_ref,Theta]

        #self.call_set_mode_service('GUIDED') #test
        
    def waypoints_recieved(self):
        if len(self.equations) != 0:
            return True
        else: 
            return False
        
    # Add functionality that notices when at final waypoint     
    def waypoint_reached(self):
        if abs(self.x0[0]-self.x_ref_ned[self.current_ref]) < 5.0 and abs(self.x0[1]-self.y_ref_ned[self.current_ref]) < 5.0 and self.current_ref < len(self.x_ref_ned)-2: 
            return True
        else: 
            return False
    
    def get_waypoint_direction(self):
        if self.x_ref_ned[self.current_ref+1]- self.x_ref_ned[self.current_ref] > 0:
            return 'right'
        else:
            return 'left'


    def run_mpc(self):

        if not self.waypoints_recieved():
            return

        mpc_start_time = time.time()
        current_time = self.mpc_iter*self.h
       
        current_equation = self.equations[self.current_ref-1]
        current_waypoint_x = self.x_ref_ned[self.current_ref]
        current_waypoint_y = self.y_ref_ned[self.current_ref]
        last_waypoint_x = self.x_ref_ned[self.current_ref]
        last_waypoint_y = self.y_ref_ned[self.current_ref]

        self.args['p'][:3] = self.x0  # set the values of the parameters vector
        waypoint_direction = self.get_waypoint_direction()

        if self.waypoint_reached():
            self.mpc_iter = 0
            self.call_set_waypoint_service(self.current_ref+1)
            print(f"eq: x = 0.5* t")
            print(f"eq: y = 0.5*t*{current_equation[0]} + {current_equation[1]}")
            
        for k in range(self.N):
            if waypoint_direction == 'right':
                t_predict = 0.5*(current_time + last_waypoint_x + k*self.h) 
                x_ref = t_predict
                y_ref = t_predict*current_equation[0] + current_equation[1]
                theta_ref = self.xs[2]
                if x_ref >= current_waypoint_x:
                    x_ref = current_waypoint_x
                    y_ref = current_waypoint_y
                    theta_ref = 0
            else:
                t_predict = -(current_time) + last_waypoint_x + k*self.h 
                x_ref = t_predict 
                y_ref = t_predict*current_equation[0] + current_equation[1] 
                theta_ref = self.xs[2]
                if x_ref <= current_waypoint_x:
                    x_ref = current_waypoint_x
                    y_ref = current_waypoint_y
                    theta_ref = 0
        
        
        self.args['p'][3*k + 3:3*k + 6] =[x_ref,y_ref, theta_ref]            
        
        self.args['x0'] = np.concatenate([self.X0.reshape(-1, 1), self.u0.reshape(-1, 1)])  # initial value of optimization variables
        sol = self.solver(x0=self.args['x0'], lbx=self.args['lbx'], ubx=self.args['ubx'], lbg=self.args['lbg'], ubg=self.args['ubg'], p=self.args['p'])
        u = np.reshape(sol['x'][3 * (self.N + 1):], (self.N, 2)).T  # get controls only from the solution
        
        
        #shift
        st = self.x0
        con = u[:, 0] # Reshape to column vector   

        f_value = self.f(st, con)
        st = st + self.h * f_value
        self.x0 = st.full().flatten()  # Convert to NumPy array and flatten
        
        self.u0 = np.vstack((u[1:], u[-1]))  # Shift the control inputs

        self.mpc_iter += 1

        self.v = float(u[0][0])
        self.omega = float(u[1][0])
        mpc_end_time = time.time()

        mpc_time = round(mpc_end_time - mpc_start_time,2)     
        
        ### DEBUGGING ###    
        # print(f"v: {self.v} , omega: {self.omega}")
        print(f"x pos: {self.x0[0]} , y pos: {self.x0[1]}")
        print(f"x ref: {x_ref} , y ref: {y_ref}")
        #print(f"start x: {self.x_ref_ned[self.current_ref-1]}")
        #print(f"current ref: {self.current_ref}")
        #print(f"mpc_time: {mpc_time}")  
        
        
        # Publish linear velocity and yaw_rate
        cmd = PositionTarget()
        cmd.header = Header()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.coordinate_frame = PositionTarget.FRAME_BODY_NED
        cmd.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                        PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                        PositionTarget.IGNORE_YAW

        cmd.velocity.x = self.v
        cmd.yaw_rate = self.omega
        self.cmd_vel_pub_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    MPC = MPCNode()

    rclpy.spin(MPC)

    rclpy.shutdown()

if __name__== '__main__':
    main()


