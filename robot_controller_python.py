# Import library untuk Webots dan fuzzy logic
from controller import Robot
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Inisialisasi robot
robot = Robot()

# Inisialisasi sensor
ir0 = robot.getDevice('ir0')
ir0.enable(time_step)  # menambahkan perintah enable untuk sensor ir0
ir1 = robot.getDevice('ir1')
ir1.enable(time_step)  # menambahkan perintah enable untuk sensor ir1
ir2 = robot.getDevice('ir2')
ir2.enable(time_step)  # menambahkan perintah enable untuk sensor ir2
ir3 = robot.getDevice('ir3')
ir3.enable(time_step)  # menambahkan perintah enable untuk sensor ir3

# Inisialisasi variabel input dan output fuzzy logic
sensors = [ir0, ir1, ir2, ir3]
error = ctrl.Antecedent(np.arange(-3, 3, 0.1), 'error')
delta_error = ctrl.Antecedent(np.arange(-3, 3, 0.1), 'delta_error')
speed = ctrl.Consequent(np.arange(-100, 100, 0.1), 'speed')

# Fuzzifikasi
error['left'] = fuzz.trapmf(error.universe, [-3, -3, -1.5, -0.5])
error['center'] = fuzz.trimf(error.universe, [-1, 0, 1])
error['right'] = fuzz.trapmf(error.universe, [0.5, 1.5, 3, 3])

delta_error['left'] = fuzz.trapmf(delta_error.universe, [-3, -3, -1.5, -0.5])
delta_error['center'] = fuzz.trimf(delta_error.universe, [-1, 0, 1])
delta_error['right'] = fuzz.trapmf(delta_error.universe, [0.5, 1.5, 3, 3])

speed['slow'] = fuzz.trapmf(speed.universe, [-100, -100, -50, 0])
speed['medium'] = fuzz.trimf(speed.universe, [-50, 0, 50])
speed['fast'] = fuzz.trapmf(speed.universe, [0, 50, 100, 100])

# Rule-base
rule1 = ctrl.Rule(error['left'] & delta_error['left'], speed['fast'])
rule2 = ctrl.Rule(error['left'] & delta_error['center'], speed['medium'])
rule3 = ctrl.Rule(error['left'] & delta_error['right'], speed['slow'])
rule4 = ctrl.Rule(error['center'] & delta_error['left'], speed['fast'])
rule5 = ctrl.Rule(error['center'] & delta_error['center'], speed['medium'])
rule6 = ctrl.Rule(error['center'] & delta_error['right'], speed['fast'])
rule7 = ctrl.Rule(error['right'] & delta_error['left'], speed['slow'])
rule8 = ctrl.Rule(error['right'] & delta_error['center'], speed['medium'])
rule9 = ctrl.Rule(error['right'] & delta_error['right'], speed['fast'])

# Defuzzifikasi
speed_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])
speed_output = ctrl.ControlSystemSimulation(speed_ctrl)

# Loop utama
while robot.step(64) != -1:
    # Membaca nilai sensor
    sensor_values = [sensor.getValue() for sensor in sensors]
    # Menghitung error dan delta_error
    error_value = (sensor_values[0] + sensor_values[1] - sensor_values[2] - sensor_values[3]) / 4
    delta_error_value = error_value - (error_value_prev if 'error_value_prev' in locals() else 0)
    error_value_prev = error_value
    
# Memasukkan input ke dalam kontrol fuzzy
speed_output.input['error'] = error_value
speed_output.input['delta_error'] = delta_error_value

# Menjalankan kontrol fuzzy
speed_output.compute()

# Membaca output kecepatan
speed_value = speed_output.output['speed']

# Mengatur kecepatan roda
left_speed = speed_value
right_speed = speed_value

# Menerapkan kecepatan pada roda
robot.setSpeed(left_speed, right_speed)
