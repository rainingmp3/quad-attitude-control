import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Animator:
    """
    Animator class plays 3D animation of drone over simulation.
    
    """
    def __init__(self, drone):
        """
        Initialize Animator class.
        Args:
            drone: drone object to animate 
        """
        # Define drone object to draw
        self.drone = drone
        self.length = self.drone.arm_length
        self.time_step = self.drone.time_step
        self.points_traj = []
        
    def init_anime(self):
        """
        init_anime is inner function for matplotlibs FuncAnimation
        class.
        Initializes drone body containers where later drones 
        position and attitude are going to be stored for drawing.

        Returns:
            Line objets for FuncAnimation.
        """
        self.point.set_data([],[])
        self.points_traj.set_data([],[])
        self.arm1.set_data([],[])
        self.arm2.set_data([],[])
        self.arm3.set_data([],[])
        self.arm4.set_data([],[])
        self.arm1.set_3d_properties([])
        ### Cube
        self.cube1.set_data([],[])
        self.cube2.set_data([],[])
        self.cube3.set_data([],[])
        self.cube4.set_data([],[])
        self.cube1.set_3d_properties([])
        self.cube2.set_3d_properties([])
        self.cube3.set_3d_properties([])
        self.cube4.set_3d_properties([])
       
        self.cube5.set_data([],[])
        self.cube6.set_data([],[])
        self.cube7.set_data([],[])
        self.cube8.set_data([],[])
        self.cube5.set_3d_properties([])
        self.cube6.set_3d_properties([])
        self.cube7.set_3d_properties([])
        self.cube8.set_3d_properties([])
        
        self.cube9.set_data([],[])
        self.cube10.set_data([],[])
        self.cube11.set_data([],[])
        self.cube12.set_data([],[])
        self.cube9.set_3d_properties([])
        self.cube10.set_3d_properties([])
        self.cube11.set_3d_properties([])
        self.cube12.set_3d_properties([])
        
        ### End of cube
        self.arm1.set_3d_properties([])
        self.arm2.set_3d_properties([])
        self.arm3.set_3d_properties([])
        self.arm4.set_3d_properties([])
        self.point.set_3d_properties([])
        self.points_traj.set_3d_properties([])  # Fixed typo: was _tpoints_traj
        return self.point, self.arm1, self.arm2, self.arm3, self.arm4, self.points_traj,self.cube1,self.cube2,self.cube3,self.cube4,\
               self.cube5,self.cube6,self.cube7,self.cube8,\
               self.cube9,self.cube10,self.cube11,self.cube12,


    def animate(self, i):
        """
        animate is inner function for matplotlibs FuncAnimation
        class.
        Populates created by init_anime containers with positional data
        for drawing.

        Returns:
            Updated objects for FuncAnimation.
        """
        x = self.drone.states_log[i][0]
        y = self.drone.states_log[i][1]
        z = self.drone.states_log[i][2]
        
        # Fixed trajectory extraction
        if i > 0:
            x_traj = [state[0] for state in self.drone.states_log[:i]]
            y_traj = [state[1] for state in self.drone.states_log[:i]]
            z_traj = [state[2] for state in self.drone.states_log[:i]]
        else:
            x_traj, y_traj, z_traj = [], [], []
        ###
        
        r_b2w = self.drone.rotation_matrix_log[i]
        time_elapsed = self.drone.time_log[i]

        # Lock animation on drone while it flies:
        lim = 5      # [m]
        self.ax.set_xlim(  x-lim, x + lim)   
        self.ax.set_ylim( y -lim, y + lim)   
        self.ax.set_zlim(  - lim,lim)   

        # if abs(x - lim) >= lim:
        #     self.ax.set_xlim(x - lim, x + lim) 
        # if abs(y - lim) >= lim:
        #     self.ax.set_ylim(y - lim, y + lim) 
        # if abs(z - lim) >= lim:
        #     self.ax.set_zlim(z - lim, z + lim) 

        # Uses rotation matrix to rotate drones body in world frame. 
        length = self.length
        rot_45 = (np.array([[np.cos(np.pi * -45 /180), -np.sin(np.pi * -45 / 180),  0],
                            [np.sin(np.pi * -45 /180),  np.cos(np.pi * -45 / 180),  0],
                            [                      0,                         0,  1]]))
        
        arm1_end = r_b2w @ rot_45 @ np.array([length, 0, 0])
        arm2_end = r_b2w @ rot_45 @ np.array([0, length, 0])
         ### Cube
        half_cube = (self.drone.cube_height/2) * 3


        self.arm1.set_data([x, x + arm1_end[0]], [y , y + arm1_end[1]])
        self.arm1.set_3d_properties([z, z + arm1_end[2]])
        self.arm2.set_data([x, x + arm2_end[0]], [y, y + arm2_end[1]])
        self.arm2.set_3d_properties([z , z + arm2_end[2]])
        self.arm3.set_data([x-arm1_end[0], x], [y - arm1_end[1], y])
        self.arm3.set_3d_properties([z- arm1_end[2], z])
        self.arm4.set_data([x - arm2_end[0], x], [y - arm2_end[1], y])
        self.arm4.set_3d_properties([z - arm2_end[2], z ])
        ### Cube
        #upper
        self.cube1.set_data([x - half_cube, x + half_cube], [y + half_cube ,y + half_cube])
        self.cube1.set_3d_properties([z,z ])
   
        self.cube2.set_data([x - half_cube, x + half_cube], [y - half_cube ,y - half_cube])
        self.cube2.set_3d_properties([z,z ])
        
        self.cube3.set_data([x - half_cube, x - half_cube], [y - half_cube ,y + half_cube])
        self.cube3.set_3d_properties([z,z ])
        
        self.cube4.set_data([x + half_cube, x + half_cube], [y - half_cube ,y + half_cube])
        self.cube4.set_3d_properties([z,z ])

        #lower
        self.cube5.set_data([x - half_cube, x + half_cube], [y + half_cube ,y + half_cube])
        self.cube5.set_3d_properties([z - 2*half_cube,z - 2*half_cube ])
   
        self.cube6.set_data([x - half_cube, x + half_cube], [y - half_cube ,y - half_cube])
        self.cube6.set_3d_properties([z-2*half_cube ,z-2*half_cube ])
        
        self.cube7.set_data([x - half_cube, x - half_cube], [y - half_cube ,y + half_cube])
        self.cube7.set_3d_properties([z-2*half_cube ,z -2*half_cube ])
        
        self.cube8.set_data([x + half_cube, x + half_cube], [y - half_cube ,y + half_cube])
        self.cube8.set_3d_properties([z-2*half_cube ,z-2*half_cube  ])
        
        
        # Colons
        self.cube9.set_data([x - half_cube, x - half_cube], [y + half_cube ,y + half_cube])
        self.cube9.set_3d_properties([z - 2*half_cube,z ])
   
        self.cube10.set_data([x - half_cube, x - half_cube], [y - half_cube ,y - half_cube])
        self.cube10.set_3d_properties([z-2*half_cube ,z ])
        
        self.cube11.set_data([x + half_cube, x + half_cube], [y - half_cube ,y - half_cube])
        self.cube11.set_3d_properties([z-2*half_cube ,z ])
        
        self.cube12.set_data([x + half_cube, x + half_cube], [y + half_cube ,y + half_cube])
        self.cube12.set_3d_properties([z-2*half_cube ,z])


    # self.cube2.set_data([x, x + cube2~ cube2_end[2], z ])
        ### End of cube
        self.point.set_data([x], [y])
        self.point.set_3d_properties([z])
        
        # Fixed trajectory plotting
        self.points_traj.set_data(x_traj, y_traj)
        self.points_traj.set_3d_properties(z_traj)

        self.ax.legend([self.point],[f"Time:{time_elapsed:.2f} s"])
        
        return self.arm1, self.arm2, self.arm3, self.arm4, self.point, self.points_traj, self.cube1, self.cube2, self.cube3, self.cube4,\
        self.cube5, self.cube6, self.cube7, self.cube8,\
        self.cube9, self.cube10, self.cube11, self.cube12 
    
    def play(self, repeat):
        """
        def play runs the animation of the drone.
        Call it in main.py
        """
        self.fig = plt.figure(figsize=(6, 4))   
        self.ax = self.fig.add_subplot(111, projection= '3d')
        # self.ax.view_init(elev= 90 , azim  = -270)
        
        self.ax.set_xlabel("X[m]")
        self.ax.set_ylabel("Y[m]")
        self.ax.set_zlabel("Z[m]")
        
        self.point, = self.ax.plot([], [], [], 'ro', label='Drone', markersize =1)
        # Cube 
        self.cube1, = self.ax.plot([], [], [], color='black', linewidth=1)
        self.cube2, = self.ax.plot([], [], [], color='black', linewidth=1)
        self.cube3, = self.ax.plot([], [], [], color='black', linewidth=1)
        self.cube4, = self.ax.plot([], [], [], color='black', linewidth=1)
        self.cube5, = self.ax.plot([], [], [], color='black', linewidth=1)
        self.cube6, = self.ax.plot([], [], [], color='black', linewidth=1)
        self.cube7, = self.ax.plot([], [], [], color='black', linewidth=1)
        self.cube8, = self.ax.plot([], [], [], color='black', linewidth=1)
        self.cube9, = self.ax.plot([], [], [], color='black', linewidth=1)
        self.cube10, = self.ax.plot([], [], [], color='black', linewidth=1)
        self.cube11, = self.ax.plot([], [], [], color='black', linewidth=1)
        self.cube12, = self.ax.plot([], [], [], color='black', linewidth=1)

        
        self.arm1, = self.ax.plot([], [], [], color='red', linewidth=2)
        self.arm2, = self.ax.plot([], [], [], color='blue', linewidth=2)
        self.arm3, = self.ax.plot([], [], [], color='purple', linewidth=2)
        self.arm4, = self.ax.plot([], [], [], color='cyan', linewidth=2)



        self.points_traj, = self.ax.plot([], [], [], 'b-', alpha=0.6, linewidth=1, label='Trajectory')

        self.anime = FuncAnimation(self.fig,
                        self.animate,
                        frames=len(self.drone.states_log),
                        interval=self.drone.time_elapsed/len(self.drone.states_log) * 200,
                        blit = False,
                        init_func = self.init_anime,
                        repeat=repeat)
        plt.show()