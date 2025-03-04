import socket

def send_script_to_openmv(script_path, ip, port):
    """ Send a Python script to the OpenMV camera for execution.

    Args:
    script_path (str): The file path of the Python script to send.
    ip (str): The IP address of the OpenMV camera.
    port (int): The port number to connect to on the OpenMV camera.
    """
    with open(script_path, 'r') as file:
        script = file.read()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ip, port))
        s.sendall(script.encode())

# Usage
openmv_ip = '192.168.4.16'  # Replace with your OpenMV camera's IP address
openmv_port = 8081         # Replace with the port you've set for script execution
script_path = '/Users/pranathipoojary/Imperial/ALBR/RoboCop/assignment_2_pt2_obstacle/servos.py'  # Path to the Python script you want to send

send_script_to_openmv(script_path, openmv_ip, openmv_port)
