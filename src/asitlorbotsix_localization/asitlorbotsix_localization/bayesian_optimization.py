#!/usr/bin/env python3

# import pandas as pd
# import numpy as np

# # Load the Excel data into a DataFrame
# data = pd.read_excel("/home/asimkumar/asitlor_ws/data4.xlsx")

# # Extract ground truth state variables for the first 50 rows
# x_gt_batch = data[['GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z','GT_Roll','GT_Pitch','GT_Yaw','GT_Vel_X','GT_Vel_Y','GT_Vel_Z','GT_Vel_Roll','GT_Vel_Pitch','GT_Angular_Vel_Yaw','GT_Accel_X','GT_Accel_Y','GT_Accel_Z']][:50].values

# # Extract estimated state variables for the first 50 rows
# x_est_batch = data[['ET_Pos_X', 'ET_Pos_Y', 'ET_Pos_Z','ET_Roll','ET_Pitch','ET_Yaw','ET_Vel_X','ET_Vel_Y','ET_Vel_Z','ET_Vel_Roll','ET_Vel_Pitch','ET_Angular_Vel_Yaw','ET_Accel_X','ET_Accel_Y','ET_Accel_Z']][:50].values

# e_x_batch = x_gt_batch - x_est_batch

# print(e_x_batch.shape)
# objective_value = 0
# # Initialize NEES_batch for the current batch
# NEES_batch = np.zeros(50)
# process_noise_covariance_matrix = np.array(process_noise_covariance_matrix).reshape((15, 15))
# # Loop over each row in the batch
# for j in range(50):
#     # Compute NEES for the current row
#     NEES_row = e_x_batch[j] @ np.linalg.inv(process_noise_covariance_matrix) @ e_x_batch[j]
#     NEES_batch[j] = NEES_row
        
# # Compute the average NEES for the current batch
# avg_NEES_batch = np.mean(NEES_batch)
# print("Avg NEES batch: ",avg_NEES_batch)
# # Compute the first term in the objective function
# term1 = np.log(avg_NEES_batch/(2*9.488))
# objective_value += np.abs(term1)
        
# # Compute the second term in the objective function
# term2 = np.abs(np.log((avg_NEES_batch - (1/avg_NEES_batch))**2 / (2 * 9.488)))
# objective_value += term2





import pandas as pd
import numpy as np
from numpy.linalg import det,inv
from skopt import gp_minimize
import matplotlib.pyplot as plt

# Load data from Excel sheet
data = pd.read_excel("/home/asimkumar/asitlor_ws/data4.xlsx")

# Define the objective function
def objective_function(process_noise_covariance_matrix):
    # Initialize objective function value
    objective_value = 0

    process_noise_covariance_1d = data.iloc[0, 40:].values

    # Reshape process_noise_covariance_matrix to a 15x15 matrix
    process_noise_covariance_matrix = np.array(process_noise_covariance_1d).reshape((15, 15))
    
    print(det(process_noise_covariance_matrix))
    
    # Loop over data in batches of 50
    num_batches = len(data) // 50
    
    for i in range(num_batches):
        start_idx = i * 50
        end_idx = (i + 1) * 50
        print("Number of Batches: ",num_batches)
        # Extract ground truth state variables for the current batch
        x_gt_batch = data[['GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z','GT_Roll','GT_Pitch','GT_Yaw','GT_Vel_X','GT_Vel_Y','GT_Vel_Z','GT_Vel_Roll','GT_Vel_Pitch','GT_Angular_Vel_Yaw','GT_Accel_X','GT_Accel_Y','GT_Accel_Z']][start_idx:end_idx].values
        
        # Extract estimated state variables for the current batch
        x_est_batch = data[['ET_Pos_X', 'ET_Pos_Y', 'ET_Pos_Z','ET_Roll','ET_Pitch','ET_Yaw','ET_Vel_X','ET_Vel_Y','ET_Vel_Z','ET_Vel_Roll','ET_Vel_Pitch','ET_Angular_Vel_Yaw','ET_Accel_X','ET_Accel_Y','ET_Accel_Z']][start_idx:end_idx].values
        
        # Compute estimation error for the current batch
        e_x_batch = x_gt_batch - x_est_batch
        print("e_x_batch shape: ",e_x_batch.shape)
        # print("e_x_batch: ",e_x_batch)
        # Initialize NEES_batch for the current batch
        NEES_batch = np.zeros(50)
        print("NEES_batch Shape: ",NEES_batch.shape)
        
        # Loop over each row in the batch
        for j in range(50):
            e_x_reshaped = e_x_batch[j].reshape(-1, 1)
            print("E-x_shaped: ",e_x_reshaped)
            print(process_noise_covariance_matrix)
            # Compute NEES for the current row
            # print("Determinant: ",det(e_x_batch))
            NEES_row = np.transpose(e_x_reshaped) @ np.linalg.inv(process_noise_covariance_matrix) @ e_x_reshaped
            print("NEES_row",NEES_row)
            NEES_batch[j] = NEES_row
            # print("NEES_batch: ",NEES_batch[j])
            

        
        
        # Compute the average NEES for the current batch
        avg_NEES_batch = np.mean(NEES_batch)
        print("Avg NEES batch: ",avg_NEES_batch)
        # Compute the first term in the objective function
        term1 = np.log(avg_NEES_batch/(2*9.488))
        objective_value += np.abs(term1)
        
        # Compute the second term in the objective function
        term2 = np.abs(np.log((avg_NEES_batch - (1/avg_NEES_batch))**2 / (2 * 9.488)))
        objective_value += term2

    # Average objective value over all batches
    objective_value /= num_batches
    print("Objective Value: ",objective_value)
    return objective_value

# Define the optimization bounds
# bounds = [(0.0000000000001,0.01)] * 225  # 15x15 matrix, all values >= 0
common_bound = (0.0000000000001, 0.00000001)
bounds = [common_bound] * 225  # Assuming a 15x15 matrix

# Modify bounds for the 96th and 112th elements
bounds[80] = (0.09, 0.1)
bounds[96] = (0.09, 0.1)  # Adjusting for zero-based index
bounds[112] = (0.09, 0.1)  # Adjusting for zero-based index
bounds[192] = (0.09, 0.1)

# Perform Bayesian optimization
res = gp_minimize(objective_function, bounds, acq_func="EI", n_calls=20, random_state=42)

# Extract the optimized process noise covariance matrix
optimized_process_noise_covariance_matrix = np.array(res.x).reshape((15, 15))


# Print the optimized process noise covariance matrix
# print("Optimized Process Noise Covariance Matrix:")
# print(optimized_process_noise_covariance_matrix)




# import pandas as pd
# import numpy as np

# # Load data from Excel sheet
# data = pd.read_excel("/home/asimkumar/asitlor_ws/data4.xlsx")

# # Extract ground truth state variables
# x_gt = data[['GT_Pos_X', 'GT_Pos_Y', 'GT_Pos_Z','GT_Roll','GT_Pitch','GT_Yaw','GT_Vel_X','GT_Vel_Y','GT_Vel_Z','GT_Vel_Roll','GT_Vel_Pitch','GT_Angular_Vel_Yaw','GT_Accel_X','GT_Accel_Y','GT_Accel_Z']].values

# # Extract estimated state variables
# x_est = data[['ET_Pos_X', 'ET_Pos_Y', 'ET_Pos_Z','ET_Roll','ET_Pitch','ET_Yaw','ET_Vel_X','ET_Vel_Y','ET_Vel_Z','ET_Vel_Roll','ET_Vel_Pitch','ET_Angular_Vel_Yaw','ET_Accel_X','ET_Accel_Y','ET_Accel_Z']].values

# # Compute estimation error
# e_x = x_gt - x_est
# print(e_x)
# # Extract process noise covariance array as 1D array
# process_noise_covariance_1d = data.iloc[0, 40:].values  # Exclude the first 7 columns and columns beyond index 231

# # Convert 1D array to 15x15 matrix
# process_noise_covariance_matrix = np.array(process_noise_covariance_1d).reshape((15, 15))

# # Print the process noise covariance matrix
# print("Process Noise Covariance Matrix:")
# print(process_noise_covariance_matrix)

# # Compute NEES
# NEES = np.sum(e_x @ process_noise_covariance_matrix * e_x, axis=1)

# # Print NEES values
# print("NEES values:")
# print(NEES)


# import pandas as pd
# import numpy as np
# import GPyOpt
# import GPy 
# from GPyOpt.models import BOModel
# import matplotlib.pyplot as plt  # Optional for plotting

# # Load data from Excel sheet
# data = pd.read_excel("/home/asimkumar/asitlor_ws/data3.xlsx", usecols=[0, 1, 2, 3, 4, 5, 6, 7])
# print("Shape of data: ", data.shape)

# # Extract parameter values and other relevant data
# X_init = data[['A_6', 'B_7', 'C_8', 'D_12', 'E_13']].values
# e_x = data['e_x'].values.reshape(-1, 1)  # Reshape to (n, 1)
# S_x = data['S_x'].values.reshape(-1, 1)  # Reshape to (n, 1)

# # Define the function to compute C_NEES (without optimizing e_x and S_x)
# def compute_C_NEES(x, e_x_val, S_x_val):
#     # Extract parameter values
#     A_6, B_7, C_8, D_12, E_13 = x[:, 0], x[:, 1], x[:, 2], x[:, 3], x[:, 4]
#     # Assuming a fixed value for n_x (replace with your actual calculation if needed)
#     n_x = 9.488
#     e_x_val = np.tile(e_x_val.mean(), (X_init.shape[0], 1))
#     S_x_val = np.tile(S_x_val.mean(), (X_init.shape[0], 1))
#     log_term1 = np.abs(np.log(e_x_val / n_x))
#     log_term2 = np.abs(np.log(S_x_val / (2 * n_x)))
#     C_NEES = log_term1 + log_term2
#     return C_NEES.reshape(-1, 1)  # Return as a column vector

# # Define the search space (excluding e_x and S_x)
# bounds = [{'name': 'A_6', 'type': 'continuous', 'domain': (data['A_6'].min(), data['A_6'].max())},
#           {'name': 'B_7', 'type': 'continuous', 'domain': (data['B_7'].min(), data['B_7'].max())},
#           {'name': 'C_8', 'type': 'continuous', 'domain': (data['C_8'].min(), data['C_8'].max())},
#           {'name': 'D_12', 'type': 'continuous', 'domain': (data['D_12'].min(), data['D_12'].max())},
#           {'name': 'E_13', 'type': 'continuous', 'domain': (data['E_13'].min(), data['E_13'].max())}]

# kernel = GPy.kern.RBF(input_dim=len(bounds))

# likelihood = GPy.likelihoods.StudentT(deg_free=4)  # Set the degrees of freedom

# model = BOModel(X_init, compute_C_NEES(X_init, e_x.mean(), S_x.mean()), kernel=kernel, Y_metadata={'likelihood': likelihood})

# acquisition = GPyOpt.acquisitions.AcquisitionEI(model=model, space=None, optimizer=None, jitter=0)

# optimization_method = GPyOpt.optimization.AcquisitionOptimizer(space=None, optimizer='lbfgs', inner_optimizer='lbfgs')

# opt = GPyOpt.methods.BayesianOptimization(f=lambda x: compute_C_NEES(x, e_x.mean(), S_x.mean()), domain=bounds, X=X_init, model=model,
#                                            acquisition=acquisition,acquisition_optimizer=optimization_method)

# max_iter = 10
# opt.run_optimization(max_iter=max_iter)


# optimized_params = opt.x_opt

# # Print the results
# print("Optimized parameters (excluding e_x and S_x):")
# for i, name in enumerate(bounds):
#     print(f"\t{name['name']}: {optimized_params[i, 0]}")

# if __name__ == "__main__":
    
#     num_grid_points = 200
#     grid = np.zeros((num_grid_points, len(bounds)))
#     for i, col in enumerate(bounds):
#         grid[:, i] = np.linspace(col['domain'][0], col['domain'][1], num_grid_points)
    
#     C_NEES_grid = compute_C_NEES(grid, e_x.mean(), S_x.mean())

#     plt.figure(figsize=(8, 6))
#     plt.scatter(X_init[:, 0], C_NEES_grid.squeeze(), color='blue', label='Data points')
#     plt.plot(grid[:, 0], C_NEES_grid.squeeze(), color='red', label='C_NEES function')
#     plt.xlabel(bounds[0]['name'])  # Assuming A_6 is the first parameter for illustration
#     plt.ylabel('C_NEES')
#     plt.title('C_NEES function with Optimized Parameters (excluding e_x and S_x)')
#     plt.legend()
#     plt.show()


















# import pandas as pd
# import numpy as np
# import GPyOpt
# import matplotlib.pyplot as plt

# # Load data from Excel sheet
# data = pd.read_excel("/home/asimkumar/asitlor_ws/data3.xlsx")

# # Extract parameter values and other relevant data
# X_init = data[['A_6', 'B_7', 'C_8', 'D_12', 'E_13', 'e_x', 'S_x']].values

# # Define the function to compute C_NEES
# def compute_C_NEES(x):
#     # Extract parameter values
#     A_6, B_7, C_8, D_12, E_13, e_x, S_x = x[:, 0], x[:, 1], x[:, 2], x[:, 3], x[:, 4], x[:, 5], x[:, 6]

#     # Compute C_NEES using the provided equation
#     n_x = 7.815
#     log_term1 = np.abs(np.log(e_x / n_x))
#     log_term2 = np.abs(np.log(S_x / (2 * n_x)))
#     C_NEES = log_term1 + log_term2

#     return np.array(C_NEES).reshape(-1, 1)  # Ensure the output has the correct shape

# # Define the search space
# bounds = [{'name': 'A_6', 'type': 'continuous', 'domain': (0.01, 0.01)},
#           {'name': 'B_7', 'type': 'continuous', 'domain': (0.01, 0.01)},
#           {'name': 'C_8', 'type': 'continuous', 'domain': (0.01, 0.01)},
#           {'name': 'D_12', 'type': 'continuous', 'domain': (0.00000000034579310345, 0.00000001)},
#           {'name': 'E_13', 'type': 'continuous', 'domain': (0.0, 1.0)},
#           {'name': 'e_x', 'type': 'continuous', 'domain': (3.0, 11.0)},  # Adjust the range as needed
#           {'name': 'S_x', 'type': 'continuous', 'domain': (605.145182391237, 1020201267880.17)}]  # Adjust the range as needed

# # Initialize Bayesian optimization
# opt = GPyOpt.methods.BayesianOptimization(f=compute_C_NEES, domain=bounds, initial_design_numdata=len(X_init), X=X_init)

# # Run optimization
# opt.run_optimization(max_iter=10)  # You can adjust the maximum number of iterations as needed

# # Retrieve optimized parameters
# optimized_params = opt.x_opt

# # Compute C_NEES for the optimized parameters
# optimized_C_NEES = compute_C_NEES(np.array([optimized_params]))

# # Print optimized parameters and C_NEES
# print("Optimized parameters:", optimized_params)
# print("Optimized C_NEES:", optimized_C_NEES)



















# def generate_numbers_with_equal_steps(start, end, n):
#     """
#     Generate 'n' numbers between 'start' and 'end' with equal steps.

#     Parameters:
#         start (float): Starting number.
#         end (float): Ending number.
#         n (int): Number of points to generate.1

#     Returns:
#         List[float]: List containing 'n' numbers between 'start' and 'end' with equal steps.
#     """
#     if n <= 1:
#         raise ValueError("Number of points (n) must be greater than 1")

#     step_size = (end - start) / (n - 1)
#     result = [start + i * step_size for i in range(n)]
#     return result

# # Example usage:
# start_num = float(input("Enter the starting number: "))
# end_num = float(input("Enter the ending number: "))
# num_points = int(input("Enter the number of points to generate: "))

# try:
#     numbers = generate_numbers_with_equal_steps(start_num, end_num, num_points)
#     print("Numbers with equal steps:")
#     print(numbers)
# except ValueError as e:
#     print(e)


# [0.1, 
#  0.09521052631578948, 
#  0.09042105263157896, 
#  0.08563157894736842, 
#  0.0808421052631579, 
#  0.07605263157894737, 
#  0.07126315789473685, 
#  0.06647368421052632, 
#  0.06168421052631579, 
#  0.05689473684210527, 
#  0.05210526315789474, 
#  0.047315789473684214, 
#  0.04252631578947369, 
#  0.03773684210526316, 
#  0.03294736842105263, 
#  0.028157894736842104, 
#  0.02336842105263158, 
#  0.018578947368421056, 
#  0.013789473684210532, 
#  0.008999999999999994]
    
# 1e-08
# 9.655206896551725e-09
# 9.310413793103448e-09
# 8.965620689655172e-09
# 8.620827586206897e-09
# 8.276034482758621e-09
# 7.931241379310344e-09
# 7.586448275862069e-09
# 7.2416551724137935e-09
# 6.896862068965517e-09
# 6.552068965517241e-09
# 6.2072758620689655e-09
# 5.862482758620689e-09
# 5.517689655172414e-09
# 5.1728965517241376e-09
# 4.828103448275862e-09
# 4.483310344827586e-09
# 4.13851724137931e-09
# 3.793724137931034e-09
# 3.448931034482758e-09
# 3.1041379310344825e-09
# 2.7593448275862062e-09
# 2.414551724137931e-09
# 2.0697586206896554e-09
# 1.7249655172413783e-09
# 1.3801724137931029e-09
# 1.0353793103448274e-09
# 6.905862068965503e-10
# 3.457931034482749e-10
# 9.999999999994796e-13

# import numpy as np
# from skopt import gp_minimize
# from skopt.space import Real
# from skopt.plots import plot_convergence

# # Given elements and their corresponding output values
# elements = [ 
#     0.00000001, 9.65520689655173E-09, 9.31041379310345E-09, 8.96562068965517E-09, 8.6208275862069E-09, 
#     8.27603448276E-09, 7.93124137931034E-09, 7.58644827586207E-09, 7.24165517241379E-09, 6.89686206896552E-09, 
#     6.55206896551724E-09, 6.20727586206897E-09, 5.86248275862069E-09, 5.51768965517241E-09, 5.17289655172414E-09, 
#     4.82810344827586E-09, 4.48331034482759E-09, 4.13851724137931E-09, 3.79372413793103E-09, 3.44893103448276E-09, 
#     3.10413793103448E-09, 2.75934482758621E-09, 2.41455172413793E-09, 2.06975862068966E-09, 1.72496551724138E-09, 
#     1.3801724137931E-09, 1.03537931034483E-09, 6.9058620689655E-10, 3.45793103448275E-10, 0.000000000001
# ]

# output_values = [
#     1.64440791354186, 1.65395558036365, 2.42984876458024, 1.62262089913694, 2.66717052647811, 1.82974057119136, 
#     2.41492394998061, 1.81240779428078, 2.21091954886931, 1.787600080004, 5.85178847417435, 4.76802277381686, 
#     2.52616850906643, 4.05611293775403, 1.84364775702078, 2.57446669459342, 2.50929878003261, 4.29912184483838, 
#     4.46522685066955, 1.89577520374927, 1.66358193135864, 2.0259476251654, 2.41954616557302, 2.24245396640746, 
#     2.25063152713009, 1.75522290433177, 4.95880209635392, 10.7543160499505, 4.80452185168909, 5.94772343517335


# ]

# # Convert the lists to numpy arrays
# elements = np.array(elements).reshape(-1, 1)
# output_values = np.array(output_values)

# # Define the parameter space for Bayesian Optimization
# space = [Real(0.000000000001,0.00000001, name='element')]

# # Define the objective function to minimize
# def objective_function(element):
#     # Find the corresponding output value for the given element
#     index = np.argmin(np.abs(elements - element))
#     return np.abs(np.log(np.sum(output_values[:index+1]) / ((index + 1) * len(output_values))))

# # Bayesian Optimization
# result = gp_minimize(
#     objective_function,
#     space,
#     n_calls=50,
#     random_state=45
# )

# # Extract the optimized parameters
# optimized_element = result.x[0]

# # Print the optimized result
# print("Optimized element:", optimized_element)

# # Plot the convergence
# plot_convergence(result)


# import numpy as np
# from skopt import gp_minimize
# from skopt.space import Real
# from skopt.plots import plot_convergence
# import matplotlib.pyplot as plt

# # Given elements and their corresponding output values
# elements = [ 
#     0.00000001, 9.65520689655173E-09, 9.31041379310345E-09, 8.96562068965517E-09, 8.6208275862069E-09, 
#     8.27603448276E-09, 7.93124137931034E-09, 7.58644827586207E-09, 7.24165517241379E-09, 6.89686206896552E-09, 
#     6.55206896551724E-09, 6.20727586206897E-09, 5.86248275862069E-09, 5.51768965517241E-09, 5.17289655172414E-09, 
#     4.82810344827586E-09, 4.48331034482759E-09, 4.13851724137931E-09, 3.79372413793103E-09, 3.44893103448276E-09, 
#     3.10413793103448E-09, 2.75934482758621E-09, 2.41455172413793E-09, 2.06975862068966E-09, 1.72496551724138E-09, 
#     1.3801724137931E-09, 1.03537931034483E-09, 6.9058620689655E-10, 3.45793103448275E-10, 0.000000000001
# ]

# output_values = [
#     1.64440791354186, 1.65395558036365, 2.42984876458024, 1.62262089913694, 2.66717052647811, 1.82974057119136, 
#     2.41492394998061, 1.81240779428078, 2.21091954886931, 1.787600080004, 5.85178847417435, 4.76802277381686, 
#     2.52616850906643, 4.05611293775403, 1.84364775702078, 2.57446669459342, 2.50929878003261, 4.29912184483838, 
#     4.46522685066955, 1.89577520374927, 1.66358193135864, 2.0259476251654, 2.41954616557302, 2.24245396640746, 
#     2.25063152713009, 1.75522290433177, 4.95880209635392, 10.7543160499505, 4.80452185168909, 5.94772343517335
# ]

# # Convert the lists to numpy arrays
# elements = np.array(elements).reshape(-1, 1)
# output_values = np.array(output_values)

# # Define the parameter space for Bayesian Optimization
# space = [Real(0.000000000001,0.00000001, name='element')]

# # Define the objective function to minimize
# def objective_function(element):
#     # Find the corresponding output value for the given element
#     index = np.argmin(np.abs(elements - element))
#     return output_values[index]

# # Bayesian Optimization
# result = gp_minimize(
#     objective_function,
#     space,
#     n_calls=50,
#     random_state=40
# )

# # Extract the optimized parameters
# optimized_element = result.x[0]
# print("Optimized element:", optimized_element)
# # Plot the scatter plot of elements and output values
# plt.figure(figsize=(10, 6))
# plt.scatter(elements, output_values, label='Original Data')
# plt.scatter(optimized_element, result.fun, color='red', label='Optimized Element')
# plt.xlabel('Element')
# plt.ylabel('Output Value')
# plt.title('Scatter Plot of Elements and Output Values')
# plt.legend()
# plt.grid(True)
# plt.show()

# # Plot the convergence
# plot_convergence(result)














#################################33
# import numpy as np
# from skopt import BayesSearchCV
# from skopt.space import Real
# from skopt.utils import use_named_args

# # Define the search space for Q values (adjust bounds as needed)
# space = [Real(1e-6, 1e2, name=f'q_{i}') for i in range(15)]  # Assuming a 15x15 matrix for state variables

# # Define the function to optimize (the negative of the performance metric)
# @use_named_args(space)
# def objective_function(**params):
#     Q_matrix = np.diag([params[f'q_{i}'] for i in range(15)])  # Assuming a diagonal covariance matrix
#     performance_metric = evaluate_filter(Q_matrix)  # Implement your filtering algorithm
#     return -performance_metric  # Minimize by maximizing the negative performance metric

# # Perform Bayesian optimization
# optimizer = BayesSearchCV(
#     objective_function,
#     search_spaces=space,
#     n_iter=50,  # Adjust the number of iterations as needed
#     random_state=42,
#     n_jobs=-1
# )

# # Run the optimization
# optimizer.fit(None)  # The first argument is not used in this case

# # Get the optimal parameters
# optimal_params = optimizer.best_params_
# optimal_Q_matrix = np.diag([optimal_params[f'q_{i}'] for i in range(15)])

# # Output the tuned process noise covariance matrix
# print("Optimal Q matrix:")
# print(optimal_Q_matrix)
