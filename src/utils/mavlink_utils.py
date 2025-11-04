
from pymavlink import mavutil
import time
from drone_commands import Command

# ---------------- CONNECTION ----------------
def connect(port="/dev/tty.usbserial-10", baud=57600):
    print(f"Connecting to {port} at {baud} baud...")
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat()
    print(f"Connected to system {master.target_system}, component {master.target_component}")
    return master

# ---------------- ARM ----------------
def arm(master):
    """Arm motors."""
    print("Arming motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Armed!")

# ---------------- DISARM ----------------
def disarm(master):
    """Disarm motors."""
    print("Disarming motors...")
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("Disarmed!")

# ---------------- FLIGHT MODES ----------------
def set_mode(master, mode_name="LOITER"):
    mode_mapping = master.mode_mapping()
    if mode_name not in mode_mapping:
        raise ValueError(f"Unknown mode: {mode_name}")
    mode_id = mode_mapping[mode_name]
    master.set_mode(mode_id)
    time.sleep(1)
    print(f"Mode set to {mode_name}")

# ---------------- TAKEOFF / LAND ----------------
def takeoff(master, altitude = 5.0):
    """Command takeoff to a specified altitude (in meters)."""
    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,  # Confirmation
    0,  # Pitch (ignored for Copter takeoff)
    0,  # Yaw (ignored for Copter takeoff)
    0,  # Latitude (ignored for Copter takeoff)
    0,  # Longitude (ignored for Copter takeoff)
    0,  # Empty
    0,
    altitude  # Desired altitude in meters
    )
    time.sleep(8)
    print("Takeoff complete (estimated).")


def land(master):
    """Land the drone."""
    print("Landing...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Landing initiated.")

# ---------------- GPS POSITION ----------------
def get_gps(master, timeout=5):
    """Get current GPS coordinates."""
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
    if not msg:
        print("No GPS data received.")
        return None
    d = msg.to_dict()
    lat = d['lat'] / 1e7
    lon = d['lon'] / 1e7
    alt = d['alt'] / 1000.0
    print(f"GPS: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}")
    return lat, lon, alt

def move_relative(master, x, y, z, vx=0, vy=0, vz=0):
    """
    Move relative to the current position/orientation (in meters).
    Positive X = forward, Positive Y = right, Positive Z = down.
    """
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1e3),        # time_boot_ms (ignored)
        master.target_system,          # target_system
        master.target_component,       # target_component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # relative to body frame
        0b0000111111111000,            # type_mask (ignore velocity/accel/yaw) only considers position (and last 4 reserved bits)
        x, y, z,                       # position (m)
        vx, vy, vz,                    # velocity (m/s)
        0, 0, 0,                       # acceleration (not used)
        0, 0)                          # yaw, yaw_rate (not used)
    
    print(f" ==== New move position ===\nx: {x:.2f}\ny: {y:.2f}\nz: {z:.2f}")


# -------------------- COMMAND HANDLER -------------------- 
def handle_command(master, cmd):

    if cmd == Command.ARM:
        arm(master)
    elif cmd == Command.DISARM:
        disarm(master)
    elif cmd == Command.TAKE_OFF:
        takeoff(master, altitude=5.0)
    elif cmd == Command.LAND:
        land(master)
    elif cmd == Command.MOVE_FORWARD:
        move_relative(master, 5, 0, 0)   # move 5m forward
    elif cmd == Command.MOVE_BACKWARD:
        move_relative(master, -5, 0, 0)  # move 5m backward
    elif cmd == Command.MOVE_RIGHT:
        move_relative(master, 0, 5, 0)   # move 5m right
    elif cmd == Command.MOVE_LEFT:
        move_relative(master, 0, -5, 0)  # move 5m left