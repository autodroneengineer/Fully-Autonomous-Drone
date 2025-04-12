# Step 1: Install MAVSDK-Python
# Before running the script, install the required library:

# bash
# Copy
# Edit
# pip install mavsdk
# Step 2: Save the Script
# Open VS Code (since you use it for coding).
# Create a new file and name it autonomous_drone.py.
# Copy and paste the script into the file.
# Save it.
# Step 3: Run the Script
# Open a terminal (Command Prompt on Windows, Terminal on macOS/Linux).

# Navigate to the folder where you saved the script:

# bash
# Copy
# Edit
# cd path/to/your/script
# Run the script:

# bash
# Copy
# Edit
# python autonomous_drone.py
# Step 4: Ensure PX4 is Running
# If you’re using a simulator (SITL), start PX4 before running the script:

# bash
# Copy
# Edit
# cd ~/PX4-Autopilot
# make px4_sitl jmavsim
# If you’re running it on a real drone, ensure Pixhawk is connected to the Raspberry Pi and running PX4 firmware.

# Step 5: Verify the Drone’s Behavior
# The script should:
# Connect to the drone.
# Wait for GPS fix.
# Arm and take off.
# Navigate through waypoints.
# Return home and land.



import asyncio

from mavsdk import System


async def autonomous_flight():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break
    
    home = await drone.telemetry.home()
    
    await drone.action.arm()
    
    await drone.action.takeoff()
    await asyncio.sleep(5)
    
    waypoints = [
        (47.397750, 8.545607, 10),
        (47.398000, 8.546000, 15),
        (47.398500, 8.546500, 20),
        (47.398750, 8.547000, 25)
    ]
    
    for lat, lon, alt in waypoints:
        await drone.action.goto_location(lat, lon, alt, 0)
        await asyncio.sleep(8)
        
        async for position in drone.telemetry.position():
            break
    
    await drone.action.return_to_launch()
    
    await drone.action.land()

asyncio.run(autonomous_flight())
