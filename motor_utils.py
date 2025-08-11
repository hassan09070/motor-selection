import csv
import os

def clean_value(value, unit=None):
    """Remove unit from value and convert to float, or convert plain number to float"""
    try:
        if value is None or value.strip() == '':
            return 0.0
        if unit:  # Only apply unit removal if a unit is specified
            return float(value.replace(unit, "").strip())
        return float(value.strip())  # Handle plain numbers (e.g., Rated RPM, Input voltage)
    except (ValueError, AttributeError):
        return 0.0

def normalize_column_name(name):
    """Normalize column name by stripping spaces and converting to lowercase"""
    return name.strip().lower()

def get_motor_specs(motor_num, torque, power):
    """Select a motor from the CSV file based on torque and power requirements"""
    try:
        # Define the CSV file path (assumed to be in the same directory)
        csv_file = "Robotic Arm - New Motor Data.csv"
        
        # Check if the CSV file exists
        if not os.path.exists(csv_file):
            print(f"CSV file {csv_file} not found")
            return {
                'motor': f"Motor {motor_num}",
                'power_rating': 0,
                'flange_size': 0,
                'voltage_type': "N/A",
                'model_name': "N/A",
                'company_name': "N/A",
                'price': 0.0,
                'motor_weight': 0.0
            }
        
        # Read the CSV file
        motor_database = []
        with open(csv_file, newline='', encoding='utf-8-sig') as f:
            reader = csv.DictReader(f)
            # Print actual column names for debugging
            print(f"CSV columns found: {reader.fieldnames}")
            
            # Define expected column names (with spaces as in your CSV)
            required_columns = {
                'Power Rating (Watts)', 'Weight (kg)', 'Rated RPM ', 'Rated Torque',
                'Input voltage', 'Voltage Type', 'Model ', 'Flange Size',
                'Company Name', 'Link'
            }
            actual_columns = set(reader.fieldnames)
            if not required_columns.issubset(actual_columns):
                missing = required_columns - actual_columns
                print(f"Missing columns in CSV: {missing}")
                return {
                    'motor': f"Motor {motor_num}",
                    'power_rating': 0,
                    'flange_size': 0,
                    'voltage_type': "N/A",
                    'model_name': "N/A",
                    'company_name': "N/A",
                    'price': 0.0,
                    'motor_weight': 0.0
                }
            
            for row in reader:
                motor_database.append({
                    'power_rating': clean_value(row['Power Rating (Watts)'], 'W'),
                    'motor_weight': clean_value(row['Weight (kg)'], 'Kg'),
                    'rated_rpm': clean_value(row['Rated RPM ']),  # Include trailing space
                    'rated_torque': clean_value(row['Rated Torque'], 'Nm'),
                    'input_voltage': clean_value(row['Input voltage']),
                    'voltage_type': row['Voltage Type'].strip(),
                    'model_name': row['Model '].strip(),  # Include trailing space
                    'flange_size': clean_value(row['Flange Size'], 'mm'),
                    'company_name': row['Company Name'].strip(),
                    'link': row['Link'].strip(),
                    'price': clean_value(row.get('Prices', '0.0'))
                })
        
        # Filter motors that meet torque and power requirements
        max_p = 100000000000000

        for motor in motor_database:
            if motor['power_rating'] < max_p and motor['power_rating'] >= power:
                max_p = motor['power_rating']
            

        suitable_motors = [
            motor for motor in motor_database
            if motor['power_rating'] == max_p
        ]
        
        if not suitable_motors:
            # If no exact match, find motors with power rating closest to but greater than required power
            suitable_motors = [
                motor for motor in motor_database
                if motor['rated_torque'] >= torque
            ]
            if not suitable_motors:
                print(f"No motor found for Motor {motor_num} with torque {torque:.3f} N⋅m and power {power:.3f} W")
                return {
                    'motor': f"Motor {motor_num}",
                    'power_rating': 0,
                    'flange_size': 0,
                    'voltage_type': "N/A",
                    'model_name': "N/A",
                    'company_name': "N/A",
                    'price': 0.0,
                    'motor_weight': 0.0
                }
            # Find the motor with the closest higher power rating
            suitable_motors = sorted(
                suitable_motors,
                key=lambda x: (x['power_rating'] - power, -x['motor_weight'])
            )
            selected_motor = suitable_motors[0]
        else:
            # If there are exact matches, select the one with the highest weight
            selected_motor = max(suitable_motors, key=lambda x: x['motor_weight'])
        
        return {
            'motor': f"Motor {motor_num}",
            'power_rating': selected_motor['power_rating'],
            'flange_size': selected_motor['flange_size'],
            'voltage_type': selected_motor['voltage_type'],
            'model_name': selected_motor['model_name'],
            'company_name': selected_motor['company_name'],
            'price': selected_motor['price'],
            'motor_weight': selected_motor['motor_weight']
        }
    
    except Exception as e:
        print(f"Error in get_motor_specs for Motor {motor_num}: {e}")
        return {
            'motor': f"Motor {motor_num}",
            'power_rating': 0,
            'flange_size': 0,
            'voltage_type': "N/A",
            'model_name': "N/A",
            'company_name': "N/A",
            'price': 0.0,
            'motor_weight': 0.0
        }
