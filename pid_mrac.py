import numpy as np

class PIDMRACMITRule:
    def __init__(self):
        self.error_bef = 0
        self.p = 0  # Initial Kp
        self.i = 0  # Initial Ki
        self.d = 0  # Initial Kd
        self.error_accum = 0
        self.gamma = 0.1  # Learning rate (step size for the MIT rule)
        
        # Batasan untuk hasil PID dan akumulasi error
        self.lower_bound = -np.pi / 2  # Batas bawah untuk hasil PID
        self.upper_bound = np.pi / 2   # Batas atas untuk hasil PID
        self.accum_lower_bound = -1.0  # Batas bawah untuk akumulasi error
        self.accum_upper_bound = 1.0   # Batas atas untuk akumulasi error
        
        # Fungsi untuk membatasi hasil PID dan akumulasi error
        self.clamp = lambda x: max(min(x, self.upper_bound), self.lower_bound)
        self.clamp_accum = lambda x: max(min(x, self.accum_upper_bound), self.accum_lower_bound)

    def set_pid(self, kp: float, ki: float, kd: float):
        '''
        Set initial PID gains (dapat disesuaikan)
        '''
        self.p = kp
        self.i = ki
        self.d = kd

    def update(self, actual_output: float, reference_output: float, dt: float) -> float:
        '''
        Update PID-MRAC controller with MIT Rule
        '''
        # Hitung error
        error = reference_output - actual_output
        self.error_accum += error  # Akumulasi error
        
        # Clamp akumulasi error agar tidak melebihi batas
        self.error_accum = self.clamp_accum(self.error_accum)

        # Penyesuaian PID menggunakan MIT Rule
        grad_kp = error  # Gradien untuk Kp
        grad_ki = self.error_accum  # Gradien untuk Ki
        grad_kd = error - self.error_bef  # Gradien untuk Kd

        # Update parameter PID berdasarkan MIT Rule
        self.p += self.gamma * grad_kp  # Update untuk Kp
        self.i += self.gamma * grad_ki  # Update untuk Ki
        self.d += self.gamma * grad_kd  # Update untuk Kd

        # Hitung output PID dengan parameter adaptif
        pid_output = self.p * error + self.i * self.error_accum + self.d * (error - self.error_bef)
        
        # Clamp output PID agar tetap dalam batas yang telah ditentukan
        pid_output = self.clamp(pid_output)

        # Update error sebelumnya
        self.error_bef = error

        return pid_output
