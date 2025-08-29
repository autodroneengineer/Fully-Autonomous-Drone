import asyncio

from mavsdk import System


async def drone_mission():
    drone = System()
    await drone.connect(system_address="udp://:14550")  # Adjust if using serial

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Position estimate ready.")
            break
    
    async for home in drone.telemetry.home():
        home_location = home
        print(f"Home position: {home_location.latitude}, {home_location.longitude}")
        break

    try:
        height = float(input("Enter the height (m) for the drone to fly: "))
    except ValueError:
        print("Invalid input. Exiting.")
        return

    print("Arming drone...")
    await drone.action.arm()

    print(f"Taking off and reaching {height} meters...")
    await drone.action.takeoff()
    await asyncio.sleep(5)  # Wait for takeoff

    print("Reaching target altitude...")
    await drone.action.goto_location(home_location.latitude, home_location.longitude, height, 0)
    await asyncio.sleep(5)  # Ensure it reaches altitude

    print("Hovering for 2 seconds...")
    await asyncio.sleep(2)
    
    print("Returning to launch (RTL)...")
    await drone.action.return_to_launch()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed successfully at home location.")
            break

async def failsafe_monitor(drone):
    async for status in drone.telemetry.status_text():
        if "Fail" in status.text or "Signal lost" in status.text:
            print("Failsafe triggered! Initiating RTL...")
            await drone.action.return_to_launch()
            break

async def main():
    drone = System()
    await drone.connect(system_address="udp://:14550")
    serial_drone = System()
    await serial_drone.connect(system_address="serial:///dev/ttyS1:57600")
    
    await asyncio.gather(drone_mission(), failsafe_monitor(drone), failsafe_monitor(serial_drone))

if __name__ == "__main__":
    asyncio.run(main())
