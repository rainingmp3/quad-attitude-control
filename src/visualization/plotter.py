import numpy as np
import matplotlib

import matplotlib.pyplot as plt



class Plotter:
    """"
    Plotter class handles states and inputs plotting over simulation.
    """
    def __init__(self, drone, controller):
        self.drone = drone
        self.controller = controller
        """
        Initialize Plotter class.
        Args:
            drone: drone object whose states are plotted
            controller: controller obj`ect whose control inputs are plotted
        """

    def plot_stats(self,show_time=3):
        """
        def plot_stats plots statistics of the whole simulation.
        Call it in main.py

        """


        time =  np.array(self.drone.time_log)
        states = np.array(self.drone.states_log)
        desired = np.array(self.controller.desired_log)
        inputs = np.array(self.controller.input_log)
        error = np.array(self.controller.error_log)

        # Drones states plotting:
        # Top 2x2 Postions

        fig, axs = plt.subplots(3, 2, figsize =(16,16),sharex=True)
       
        # axs[0, 0].plot(time, states[:,0],label="X",color="red")
        # axs[0, 0].plot(time, states[:,1],label="Y",color="green")
        axs[0, 0].plot(time, states[:,2],label="Z",color="blue")
        axs[0, 0].grid()
        axs[0, 0].set_xlim(min(time), max(time))
        axs[0, 0].legend()
        axs[0, 0].set_title("Position [m]")
        
        # axs[0, 1].plot(time, states[:,3],label="vx",color="red")
        axs[0, 1].plot(time, states[:,4],label="vy",color="green")
        axs[0, 1].plot(time, states[:,5],label="vz",color="blue")
        axs[0, 1].grid()
        axs[0, 1].set_xlim(min(time), max(time))
        axs[0, 1].legend()
        axs[0, 1].set_title("Velocity [m/s]")
        
        axs[1, 0].plot(time, states[:,6],label="φ",color="red")
        axs[1, 0].plot(time, states[:,7],label="θ",color="green")
        # axs[1, 0].plot(time, states[:,8],label="ψ",color="blue")
        axs[1, 0].grid()
        axs[1, 0].set_xlim(min(time), max(time))
        axs[1, 0].legend()
        axs[1, 0].set_title("Attitude [rad]")
        
        axs[1, 1].plot(time, states[:, 9],label="p",color="red")
        axs[1, 1].plot(time, states[:,10],label="q",color="green")
        # axs[1, 1].plot(time, states[:,11],label="r",color="blue")
        axs[1, 1].grid()
        axs[1, 1].set_xlim(min(time), max(time))
        axs[1, 1].legend()
        axs[1, 1].set_title("Angular velocity [rad/s]")

        # Bottom 2x2 Inputs and Errors
        # att_input = ["Σ Thrust","Moment φ","Moment θ","Moment ψ"]
        # try:
        #     axs[2,0].plot(time, inputs[:,0], label=f'Input {att_input[0]}', color = 'grey')
        #     axs[2,0].plot(time, desired[:,0], label=f'Desired {att_input[0]}', color = 'black')
        #     axs[2,0].set_title(att_input[0])
        #     axs[2,0].set_xlim(min(time), max(time))
        #     axs[2,0].legend()
        #     axs[2,0].grid()

            # axs[2,1].plot(time, inputs[:,1], label=f'Input {att_input[1]}', color = 'lime')
            # axs[2,1].plot(time, desired[:,6], label=f'Desired {att_input[1]}', color = 'green')
            # axs[2,1].set_title(att_input[1])
            # axs[2,1].set_xlim(min(time), max(time))
            # axs[2,1].legend()
            # axs[2,1].grid()
        
            # axs[3,0].plot(time, inputs[:,2], label=f'Input {att_input[2]}', color = 'magenta')
            # axs[3,0].plot(time, desired[:,7], label=f'Desired {att_input[2]}', color = 'red')
            # axs[3,0].set_title(att_input[2])
            # axs[3,0].set_xlim(min(time), max(time))
            # axs[3,0].legend()
            # axs[3,0].grid()

            # axs[3,1].plot(time, inputs[:,3], label=f'Input {att_input[3]}', color = 'cyan')
            # axs[3,1].plot(time, desired[:,8], label=f'Desired {att_input[3]}', color = 'blue')
            # axs[3,1].set_title(att_input[3])
            # axs[3,1].set_xlim(min(time), max(time))
            # axs[3,1].legend()
            # axs[3,1].grid()

        # axs[2, 0].plot(time, error[:, 0], label='eₓ', color='red')
        # axs[2, 0].plot(time, error[:, 1], label='eᵧ', color='green')
        axs[2, 0].plot(time, error[:, 2], label='e_z', color='blue')
        axs[2, 0].set_title('Position error [m]')
        axs[2, 0].set_xlim(time.min(), time.max())
        axs[2, 0].grid()
        axs[2, 0].legend()
        
        # axs[4,1].plot(time, error[:, 3], label='eₓ', color='red')
        # axs[4,1].plot(time, error[:, 4], label='eᵧ', color='green')
        axs[2,1].plot(time, error[:, 5], label='e_vz', color='blue')
        axs[2,1].set_title('Velocity error [m]')
        axs[2,1].set_xlim(time.min(), time.max())
        axs[2,1].grid()
        axs[2,1].legend()
               
               
        # except:
        #     pass 


        # plt.tight_layout()
        plt.show(block=False)
        plt.pause(show_time)
        plt.close() 