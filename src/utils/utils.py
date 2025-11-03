
import yaml

def load_config(config_file_path):

    try:
        with open(config_file_path) as file:
            config_data = yaml.safe_load(file)
        
        print("YAML data loaded successfully:")
        print(config_data)

    except FileNotFoundError:
        print(f"Error: The file '{config_file_path}' was not found.")
    except yaml.YAMLError as e:
        print(f"Error parsing YAML file: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    return config_data