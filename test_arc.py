import time
import math
# Try to import Bee, or use a Mock if running in an environment without the SDK file or serial
try:
    from DBDynamicsPro import Bee
except ImportError:
    print("DBDynamicsPro.py not found. Please ensure it is in the same directory.")
    exit(1)

# --- Configuration ---
PORT = "COM9"  # Change this to your actual serial port
BAUDRATE = 2000000

# Machine Kinematics Parameters
# Resolution: 51200 pulses corresponds to 80mm
PULSES_PER_REV = 51200
LEAD_MM = 80.0
PULSES_PER_MM = PULSES_PER_REV / LEAD_MM  # 1280.0 pulses/mm

def cartesian_kinematics(x, y, z):
    """
    Custom Kinematics for a Cartesian XY Stage.
    Axis 4 -> X
    Axis 3 -> Y
    """
    # Calculate pulses
    p_x = x * PULSES_PER_MM
    p_y = y * PULSES_PER_MM
    
    # Return dictionary {axis_id: pulse_value}
    return {
        4: int(p_x),
        3: int(p_y)
    }

def main():
    print("Initializing DBDynamicsPro SDK...")
    try:
        bee = Bee(PORT, BAUDRATE)
    except Exception as e:
        print(f"Failed to connect to hardware: {e}")
        print("Note: If you are running this test without hardware, code verification stops here.")
        return

    # 1. Setup Parameters
    start_pos = [0, 0]
    end_pos = [100, 100]
    center_pos = [100, 0]
    duration_ticks = 300  # Total interpolation points
    
    print(f"\nTest Parameters:")
    print(f"Start:  {start_pos}")
    print(f"End:    {end_pos}")
    print(f"Center: {center_pos}")
    print(f"Time:   {duration_ticks} ticks")
    print(f"Resolution: {PULSES_PER_MM} pulses/mm")

    # 2. Reset internal position to Start Point (0,0)
    # Important: The SDK tracks 'last_si_pos'. We must ensure it matches our start.
    # We manually set the internal state to (0,0,0) assuming the machine is physically there.
    # In a real scenario, you might move the machine there first.
    print("\nResetting internal position state to (0,0)...")
    bee.setLastSIPose([0, 0, 0, 0, 0, 0, 0, 0])
    
    # 3. Generate and Queue Arc Trajectory (Forward)
    print("Calculating and queueing arc trajectory (Forward)...")
    
    # Forward: (0,0) -> (100,100), Center (100,0)
    bee.setCircularArcInvK(
        dt=duration_ticks,
        start_pos_xy=start_pos,
        end_pos_xy=end_pos,
        center_pos_xy=center_pos,
        direction=None,  # Auto-detect shortest path
        kinematics_func=cartesian_kinematics
    )

    # 4. Wait / Dwell time (3 seconds)
    # Since setCircularArcInvK updates the internal queue, we can just append
    # a "stay in place" command. But the easiest way is to create a small helper
    # or just replicate the last position for N ticks.
    # Assuming communication cycle is ~10ms (check SDK implementation, usually 1 tick = 1 comm cycle)
    # If 1 tick is handled every loop, and loop has sleep(0.01) or is driven by reply...
    # Let's assume we need to fill the buffer for 3 seconds.
    # If the frequency is 100Hz (10ms), 3s = 300 ticks.
    print("Queueing 3s dwell...")
    dwell_ticks = 100
    
    # We can use setCircularArcInvK with start=end to dwell, or manually append.
    # Let's manually append to ensure clean holding.
    last_pos = bee._get_last_queue_positions() # This internal helper we added is useful
    for axis in range(8):
        bee.append_to_queue(axis, [last_pos[axis]] * dwell_ticks)

    # 5. Generate and Queue Arc Trajectory (Backward)
    print("Calculating and queueing arc trajectory (Backward)...")
    
    # Backward: (100,100) -> (0,0), Center (100,0)
    # Note: Start is now end_pos, End is now start_pos
    bee.setCircularArcInvK(
        dt=duration_ticks,
        start_pos_xy=end_pos,
        end_pos_xy=start_pos,
        center_pos_xy=center_pos,
        direction=None,  # Auto-detect shortest path
        kinematics_func=cartesian_kinematics
    )

    # 6. Verify Data in Queue (Optional Debugging)
    # Check the first few and last few points in the queue for Axis 4 and 3
    qX = bee._array_p[4]
    qY = bee._array_p[3]
    
    if len(qX) > 0:
        print(f"\nQueue Verification ({len(qX)} points total):")
        print(f"Start Point         -> X: {qX[0]}, Y: {qY[0]}")
        print(f"End of Forward      -> X: {qX[duration_ticks-1]}, Y: {qY[duration_ticks-1]}")
        print(f"End of Dwell        -> X: {qX[duration_ticks+dwell_ticks-1]}, Y: {qY[duration_ticks+dwell_ticks-1]}")
        print(f"Final Point         -> X: {qX[-1]}, Y: {qY[-1]}")
    
    # 7. Execution
    # Note: To actually move the motor, the 'linkProcess' thread automatically sends data 
    # when 'syncInterpolationFlag' is set.
    
    user_input = input("\nReady to move? (y/n): ")
    if user_input.lower() == 'y':
        print("Starting motion...")
        
        # Enable interpolation mode (assuming motors are enabled/powered)
        # You might need to set operation mode for each axis first
        bee.setInterpolationPositionMode(4)
        bee.setInterpolationPositionMode(3)
        time.sleep(0.1)
        
        # Start Sync Interpolation
        bee.syncInterpolationFlag = 1
        
        # Wait until motion completes (queue is empty)
        bee.waitSIP()
        
        # Stop Interpolation
        bee.syncInterpolationFlag = 0
        print("Motion complete.")
    else:
        print("Motion skipped.")

    bee.stop()

if __name__ == "__main__":
    main()
