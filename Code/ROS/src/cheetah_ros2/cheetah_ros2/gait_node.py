#!/usr/bin/env python3
import rclpy
from enum import Enum
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64

from cheetah_ros2.linear_mpc_configs import LinearMpcConfig

# shebang to make executable with python3 interpreter
# importing node class and other libraries

class Gait(Enum):
    # phase offset of each leg
    # 16 is length of one complete step cycle
    # each leg spends 16/4 = 4 segments in stance and 12 segments in swing, with different offsets for each leg
    # gait pattern (FL,FR,BL,BR)
    CRAWL16 = 'crawl', 16, np.array([0, 8, 4, 12]), np.array([12, 12, 12, 12])
    CRAWL64 = 'crawl', 64, np.array([0, 32, 16, 48]), np.array([48, 48, 48, 48])
    CRAWL100 = 'crawl', 100, np.array([0, 50, 25, 75]), np.array([75, 75, 75, 75])
    CRAWL128 = 'crawl', 128, np.array([0, 64, 32, 96]), np.array([96, 96, 96, 96])
    CRAWL1000 = 'crawl', 1000, np.array([0, 500, 250, 750]), np.array([750, 750, 750, 750])
    CRAWL2000 = 'crawl', 2000, np.array([0, 1000, 500, 1500]), np.array([1500, 1500, 1500, 1500])
    CRAWL5000 = 'crawl', 5000, np.array([0, 2500, 1250, 3750]), np.array([3750, 3750, 3750, 3750])
    CRAWL10000 = 'crawl', 10000, np.array([0, 5000, 2500, 7500]), np.array([7500, 7500, 7500, 7500])

    # constructor function for each gait
    def __init__(self, name: str, num_segment: int, stance_offsets: np.ndarray, stance_durations: np.ndarray) -> None:
        self.__name = name
        self.__num_segment = num_segment
        self.__stance_offsets = stance_offsets
        self.__stance_durations = stance_durations

        self.__load_parameters()

        self.total_swing_time: int = num_segment - stance_durations[0] # 4
        self.total_stance_time: int = stance_durations[0] # 12
        
        # normalized offsets, for the equation mentioned for gait phase
        self.stance_offsets_normalized = stance_offsets / num_segment
        self.stance_durations_normalized = stance_durations / num_segment
        
        # NOTE: do we need to initialize self.phase, its only being init in the set_iteration function

    def __load_parameters(self) -> None:
        self.__dt_control: float = LinearMpcConfig.dt_control
        self.__iterations_between_mpc: int = LinearMpcConfig.iteration_between_mpc
        self.__mpc_horizon: int = LinearMpcConfig.horizon


    # properties to access the gait parameters (getter functions)
    @property
    def name(self) -> str: return self.__name
    @property
    def num_segment(self) -> int: return self.__num_segment
    @property
    def stance_offsets(self) -> np.ndarray: return self.__stance_offsets
    @property
    def stance_durations(self) -> np.ndarray: return self.__stance_durations
    
    # swing and stance times are calculated by multipling number of segments for swing and stance with time for mpc (freq * timesteps for mpc)
    @property
    def swing_time(self) -> float: return self.get_total_swing_time(self.__dt_control * self.__iterations_between_mpc)
    @property
    def stance_time(self) -> float: return self.get_total_stance_time(self.__dt_control * self.__iterations_between_mpc)


    # cur_iteration is main clock iteration, and we need to evaluate mpc every 20 iterations of main clock
    # so we determine how many times mpc has run in the main clock and we loop that around based on num_segment so that mpc runs 16 times for a gait cycle and loops back around in the same main loop
    # this sets what segment of the gait we are in

    def set_iteration(self, iterations_between_mpc: int, cur_iteration: int) -> None:
        # what gait iteration (mpc) is running
        self.iteration = np.floor(cur_iteration / iterations_between_mpc) % self.num_segment
        # phase remains between 0 and 1. calculated from cur_iteration being normalized with the time for mpc in all segments. so we know for each leg what phase its closer to
        
        # every 20*16=320 iterations, a new gait cycle is starting
        # and we can determine at which iteration we are currently based on the 320 iterations
        # and we can normalize to get phase
        self.phase = (cur_iteration % (iterations_between_mpc * self.num_segment)) / (iterations_between_mpc * self.num_segment)

    def get_gait_table(self) -> np.ndarray:
        # nominal gait schedule (1 - stance, 0 - swing), is a table for 16 steps and so is a bigger table
        
        # 4 legs with 1 step horizon
        gait_table = np.zeros(4 * self.__mpc_horizon, dtype=np.float32)
        # flattens the 2D table into 1D and determines if leg closer to stance then it should be in stance and same for swing.
        for i in range(self.__mpc_horizon):
            # i_horizon is the gait segment for the ith step of horizon
            i_horizon = (i + 1 + self.iteration) % self.num_segment # looking ahead to determine phase
            
            # NOTE: NEED TO REVISIT THIS LOGIC 
            cur_segment = i_horizon - self.stance_offsets # factoring in stance offsets for all legs as one moves at a time
            for j in range(4):
                if cur_segment[j] < 0:
                    cur_segment[j] += self.num_segment
                if cur_segment[j] < self.stance_durations[j]:
                    # i is future timestep and j is leg index, we check if the current timestep falls within the swing or stance duration
                    gait_table[i*4+j] = 1
                else:
                    gait_table[i*4+j] = 0
        return gait_table


    # NOTE: THIS CAN BE OPTIMIZED WITH NUMPY ARRAYS LATER

    # value between 0 and 1 to indicate how much of swing state has been completed, used for swing foot trajectory generation. 0 means just started swing, 1 means about to end swing and start stance. same for stance state.
    # this is passed into bezzier curve or cubic spline function
    def get_swing_state(self) -> np.ndarray:
        swing_offsets_normalized = self.stance_offsets_normalized + self.stance_durations_normalized
        # NOTE: since swing offset is calculated by adding stance offset and duration, it can be greater than 1, so we need to wrap it around to get the correct phase for swing state calculation
        for i in range(4):
            # NOTE: is this correct to not index the subtraction line?
            
            if(swing_offsets_normalized[i] > 1):
                swing_offsets_normalized[i] -= 1
                # self.get_logger().info(f'Leg {i} swing offset wrapped around: {swing_offsets_normalized}')
                
        swing_durations_normalized = 1 - self.stance_durations_normalized
        phase_state = np.array([self.phase, self.phase, self.phase, self.phase], dtype=np.float32)
        swing_state = phase_state - swing_offsets_normalized
        
        for i in range(4):
            # if neg
            if swing_state[i] < 0: swing_state[i] += 1 # to wrap around, calculating which phase currently in
            
            # checking if still swinging, if not then 0 (stance), else we're swinging and we calculate phase
            if swing_state[i] > swing_durations_normalized[i]: swing_state[i] = 0
            else: swing_state[i] = swing_state[i] / swing_durations_normalized[i]
        return swing_state

    def get_stance_state(self) -> np.ndarray:
        phase_state = np.array([self.phase, self.phase, self.phase, self.phase], dtype=np.float32)
        stance_state = phase_state - self.stance_offsets_normalized
        for i in range(4):
            if stance_state[i] < 0: stance_state[i] += 1
            if stance_state[i] > self.stance_durations_normalized[i]: stance_state[i] = 0
            else: stance_state[i] = stance_state[i] / self.stance_durations_normalized[i]
        return stance_state

    def get_total_swing_time(self, dt_mpc: float) -> float: return dt_mpc * self.total_swing_time
    def get_total_stance_time(self, dt_mpc: float) -> float: return dt_mpc * self.total_stance_time



class GaitControllerNode(Node):
    def __init__(self):
        super().__init__('gait_controller') # every node is a child of the Node class
        
        self.current_gait = Gait.CRAWL2000
        self.iterations_between_mpc = LinearMpcConfig.iteration_between_mpc
        
        self.dt_control = LinearMpcConfig.dt_control
        self.iteration_counter = 0
        
        self.pub_nominal = self.create_publisher(Float64MultiArray, '/nominal_schedule', 1)
        self.pub_swing_phase = self.create_publisher(Float64MultiArray, '/swing_phases', 1)
        self.pub_stance_phase = self.create_publisher(Float64MultiArray, '/stance_phases', 1)
        self.pub_stance_time = self.create_publisher(Float64, '/gait/stance_time', 1)
        self.pub_swing_time = self.create_publisher(Float64, '/gait/swing_time', 1)
        
        # Create a timer ticking at the control frequency, we dont use sleep since it would block the execution of other commands, so we create a hardware timer that calls the control loop at the specific freq
        self.timer = self.create_timer(self.dt_control, self.control_loop)
        
        # NOTE: node is created with some timer frequency, and the callback to call what function needs to run at that frequency
        hz = int(1.0 / self.dt_control)
        self.get_logger().info(f'Gait Scheduler Active: Running {self.current_gait.name} gait at {hz} Hz.')

    def control_loop(self):
        # self.get_logger().info(f'Gait Controller Iteration: {self.iteration_counter}')
        # Update the internal phase of the gait
        self.current_gait.set_iteration(self.iterations_between_mpc, self.iteration_counter)
        
        # Publish continuous phase parameters at 1000 Hz for the FSM
        swing_state = self.current_gait.get_swing_state()
        msg_swing = Float64MultiArray(data=swing_state.tolist())
        self.pub_swing_phase.publish(msg_swing)
        
        stance_state = self.current_gait.get_stance_state()
        msg_stance = Float64MultiArray(data=stance_state.tolist())
        self.pub_stance_phase.publish(msg_stance)
        
        msg_stance_time = Float64()
        msg_stance_time.data = float(self.current_gait.stance_time)
        self.pub_stance_time.publish(msg_stance_time)
        
        msg_swing_time = Float64()
        msg_swing_time.data = float(self.current_gait.swing_time)
        self.pub_swing_time.publish(msg_swing_time)
        
        # The MPC algorithm only runs every 20 iterations (50 Hz)
        if self.iteration_counter % self.iterations_between_mpc == 0:
            gait_table = self.current_gait.get_gait_table()
            
            # Extract just the first 4 elements to see the current instantaneous contact state, as mpc only applies first control input
            nominal_schedule = gait_table[0:4]
            msg_nom = Float64MultiArray(data=nominal_schedule.tolist())
            self.pub_nominal.publish(msg_nom)
            
            # self.get_logger().info(f'MPC Step | Phase: {self.current_gait.phase:.2f} | Contact (FL,FR,RL,RR): {nominal_schedule}')
        self.iteration_counter += 1



def main(args=None):
    rclpy.init(args=args) # initializing ROS2 communication
    node = GaitControllerNode() # instantiate node
    try:
        rclpy.spin(node) # indefintely wait on this node until process is killed
    except KeyboardInterrupt: # destroying and cleaning up node when ctrl + c
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()