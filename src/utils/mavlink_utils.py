
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
def takeoff(master, altitude):
    """Command takeoff to a specified altitude (in meters)."""
    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
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

# ---------------- MOVEMENT ----------------
def move_local(master, x, y, z):
    """
    Move to position (x, y, z) in LOCAL_NED frame.
    x=forward, y=right, z=down (positive down)
    """
    print(f"Moving to (x={x}, y={y}, z={z}) in LOCAL_NED frame...")
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,  # bitmask: position only
        x, y, z,
        0, 0, 0,  # velocity
        0, 0, 0,  # acceleration
        0, 0      # yaw, yaw_rate
    )


def move_relative(master, dx=0, dy=0, dz=0):
    """Move relative to current local position."""
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=3)
    if not msg:
        print("No local position data.")
        return
    pos = msg.to_dict()
    x = pos['x'] + dx
    y = pos['y'] + dy
    z = pos['z'] + dz
    print(f"Moving relative: Δx={dx}, Δy={dy}, Δz={dz}")
    move_local(master, x, y, z)


def send_ned_velocity(master, vx, vy, vz, duration=1):
    """
    Send velocity command in NED frame (m/s) for a given duration.
    vx: forward (+x), vy: right (+y), vz: down (+z)
    """
    msg = master.mav.set_position_target_local_ned_encode(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # velocity only
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0, 0, 0)
    
    for _ in range(duration):
        master.mav.send(msg)
        time.sleep(1)


# -------------------- COMMAND HANDLER -------------------- 
def handle_command(master, cmd):
    distance = 4  # meters
    speed = 1     # m/s
    duration = int(distance / speed)

    if cmd == Command.MOVE_FORWARD:
        print("⬆️ Moving forward 4 m")
        send_ned_velocity(master, speed, 0, 0, duration)
    elif cmd == Command.MOVE_BACKWARD:
        print("⬇️ Moving backward 4 m")
        send_ned_velocity(master, -speed, 0, 0, duration)
    elif cmd == Command.MOVE_LEFT:
        print("⬅️ Moving left 4 m")
        send_ned_velocity(master, 0, -speed, 0, duration)
    elif cmd == Command.MOVE_RIGHT:
        print("➡️ Moving right 4 m")
        send_ned_velocity(master, 0, speed, 0, duration)
    elif cmd == Command.LAND:
        print("🛬 Landing")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0)
    elif cmd == Command.TAKE_OFF:
        print("🚁 Taking off to 3 m")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, 3)
    elif cmd == Command.HOVER:
        print("🕹️ Hovering")
        send_ned_velocity(master, 0, 0, 0, 2)
    else:
        print("❓ Unrecognized or unsupported command.")