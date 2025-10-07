#!/usr/bin/env python3
import os
import csv
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from datetime import datetime
import numpy as np

def save_csv_data(data_dict, controller_type, world_name):
    """
    Save simulation data to CSV file in tabular format.
    
    Args:
        data_dict (dict): Dictionary containing lists of data with keys:
                         'time', 'x_pos', 'y_pos', 'yaw_pos', 'velocity_x', 
                         'velocity_y', 'velocity_yaw', 'starting_displacement',
                         'total_distance_traveled', 'x_error', 'y_error', 'yaw_error'
        controller_type (str): Type of controller ('P' or 'Enhanced_PID')
        world_name (str): Name of the world file being tested
    """
    # Create directory if it doesn't exist
    data_dir = "/home/lonebanana/lidar_testing/src/research_storage_data"
    os.makedirs(data_dir, exist_ok=True)
    
    # Generate timestamp for unique filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{controller_type}_{world_name}_data_{timestamp}.csv"
    filepath = os.path.join(data_dir, filename)
    
    # Define headers
    headers = [
        'time', 'x_pos', 'y_pos', 'yaw_pos', 
        'velocity_x', 'velocity_y', 'velocity_yaw',
        'starting_displacement', 'total_distance_traveled',
        'x_error', 'y_error', 'yaw_error'
    ]
    
    try:
        with open(filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header
            writer.writerow(headers)
            
            # Get the length of data (assuming all lists are same length)
            data_length = len(data_dict['time'])
            
            # Write data rows
            for i in range(data_length):
                row = []
                for header in headers:
                    if header in data_dict:
                        row.append(data_dict[header][i])
                    else:
                        row.append('')  # Empty if data not available
                writer.writerow(row)
        
        print(f"CSV data saved successfully to: {filepath}")
        return filepath
        
    except Exception as e:
        print(f"Error saving CSV data: {e}")
        return None

def create_position_plots(data_dict, controller_type, world_name):
    """
    Create a 2x2 matplotlib plot showing position and orientation data.
    
    Args:
        data_dict (dict): Dictionary containing data lists
        controller_type (str): Type of controller ('P' or 'Enhanced_PID')
        world_name (str): Name of the world file being tested
    """
    # Create directory if it doesn't exist
    data_dir = "/home/lonebanana/lidar_testing/src/research_storage_data"
    os.makedirs(data_dir, exist_ok=True)
    
    # Generate timestamp for unique filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{controller_type}_{world_name}_position_plots_{timestamp}.png"
    filepath = os.path.join(data_dir, filename)
    
    try:
        # Create 2x2 subplot
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle(f'{controller_type} Controller - {world_name} World\nPosition and Orientation Analysis', 
                     fontsize=14, fontweight='bold')
        
        # Plot 1: y_pos vs x_pos (trajectory)
        axes[0, 0].plot(data_dict['x_pos'], data_dict['y_pos'], 'b-', linewidth=2, alpha=0.7)
        axes[0, 0].scatter(data_dict['x_pos'][0], data_dict['y_pos'][0], 
                          color='green', s=100, marker='o', label='Start', zorder=5)
        axes[0, 0].scatter(data_dict['x_pos'][-1], data_dict['y_pos'][-1], 
                          color='red', s=100, marker='s', label='End', zorder=5)
        axes[0, 0].set_xlabel('X Position (m)')
        axes[0, 0].set_ylabel('Y Position (m)')
        axes[0, 0].set_title('Robot Trajectory (Y vs X)')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()
        axes[0, 0].set_aspect('equal', adjustable='box')
        
        # Plot 2: x_pos vs time
        axes[0, 1].plot(data_dict['time'], data_dict['x_pos'], 'r-', linewidth=2)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('X Position (m)')
        axes[0, 1].set_title('X Position vs Time')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Plot 3: y_pos vs time
        axes[1, 0].plot(data_dict['time'], data_dict['y_pos'], 'g-', linewidth=2)
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Y Position (m)')
        axes[1, 0].set_title('Y Position vs Time')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Plot 4: yaw_pos vs time
        # Convert yaw to degrees for better readability
        yaw_degrees = [np.degrees(yaw) for yaw in data_dict['yaw_pos']]
        axes[1, 1].plot(data_dict['time'], yaw_degrees, 'm-', linewidth=2)
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Yaw Position (degrees)')
        axes[1, 1].set_title('Yaw Position vs Time')
        axes[1, 1].grid(True, alpha=0.3)
        
        # Adjust layout to prevent overlap
        plt.tight_layout()
        
        # Save the plot
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        plt.close()  # Close to free memory
        
        print(f"Position plots saved successfully to: {filepath}")
        return filepath
        
    except Exception as e:
        print(f"Error creating position plots: {e}")
        return None

def create_error_plots(data_dict, controller_type, world_name):
    """
    Create overlapping plot of x_error, y_error, and yaw_error over time.
    
    Args:
        data_dict (dict): Dictionary containing data lists
        controller_type (str): Type of controller ('P' or 'Enhanced_PID')
        world_name (str): Name of the world file being tested
    """
    # Create directory if it doesn't exist
    data_dir = "/home/lonebanana/lidar_testing/src/research_storage_data"
    os.makedirs(data_dir, exist_ok=True)
    
    # Generate timestamp for unique filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{controller_type}_{world_name}_error_plots_{timestamp}.png"
    filepath = os.path.join(data_dir, filename)
    
    try:
        # Create single plot
        fig, ax = plt.subplots(1, 1, figsize=(12, 8))
        
        # Plot errors with different colors and styles
        ax.plot(data_dict['time'], data_dict['x_error'], 'r-', linewidth=2.5, 
                label='X Error', alpha=0.8)
        ax.plot(data_dict['time'], data_dict['y_error'], 'b-', linewidth=2.5, 
                label='Y Error', alpha=0.8)
        
        # Convert yaw error to degrees for better readability
        yaw_error_degrees = [np.degrees(yaw_err) for yaw_err in data_dict['yaw_error']]
        ax.plot(data_dict['time'], yaw_error_degrees, 'g-', linewidth=2.5, 
                label='Yaw Error (degrees)', alpha=0.8)
        
        # Formatting
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Error', fontsize=12)
        ax.set_title(f'{controller_type} Controller - {world_name} World\nTracking Errors Over Time', 
                     fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=11, loc='best')
        
        # Add zero line for reference
        ax.axhline(y=0, color='black', linestyle='--', alpha=0.5, linewidth=1)
        
        # Add statistics box
        x_rms = np.sqrt(np.mean([x**2 for x in data_dict['x_error']]))
        y_rms = np.sqrt(np.mean([y**2 for y in data_dict['y_error']]))
        yaw_rms = np.sqrt(np.mean([y**2 for y in yaw_error_degrees]))
        
        stats_text = f'RMS Errors:\nX: {x_rms:.4f} m\nY: {y_rms:.4f} m\nYaw: {yaw_rms:.2f}°'
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=10,
                verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Adjust layout
        plt.tight_layout()
        
        # Save the plot
        plt.savefig(filepath, dpi=300, bbox_inches='tight')
        plt.close()  # Close to free memory
        
        print(f"Error plots saved successfully to: {filepath}")
        return filepath
        
    except Exception as e:
        print(f"Error creating error plots: {e}")
        return None

# Example usage function (for testing purposes)
def example_usage():
    """
    Example of how to use the functions with sample data.
    This function can be removed in production.
    """
    # Sample data structure
    sample_data = {
        'time': [i * 0.1 for i in range(100)],
        'x_pos': [i * 0.01 for i in range(100)],
        'y_pos': [np.sin(i * 0.1) for i in range(100)],
        'yaw_pos': [i * 0.05 for i in range(100)],
        'velocity_x': [0.1] * 100,
        'velocity_y': [0.05] * 100,
        'velocity_yaw': [0.02] * 100,
        'starting_displacement': [0.5] * 100,
        'total_distance_traveled': [i * 0.015 for i in range(100)],
        'x_error': [0.1 * np.sin(i * 0.2) for i in range(100)],
        'y_error': [0.05 * np.cos(i * 0.3) for i in range(100)],
        'yaw_error': [0.1 * np.sin(i * 0.15) for i in range(100)]
    }
    
    # Test all functions
    save_csv_data(sample_data, "Enhanced_PID", "test_world")
    create_position_plots(sample_data, "Enhanced_PID", "test_world")
    create_error_plots(sample_data, "Enhanced_PID", "test_world")

if __name__ == "__main__":
    # Run example if script is executed directly
    example_usage() #Fun testing