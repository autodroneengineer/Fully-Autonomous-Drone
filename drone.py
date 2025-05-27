import asyncio

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan


async def drone_mission(drone):
    await drone.connect(system_address="udp://:14550")

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

    # Get home location
    async for home in drone.telemetry.home():
        home_location = home
        print(f"Home position: {home_location.latitude}, {home_location.longitude}")
        break

    try:
        height = float(input("Enter the height (m) for the drone to fly at each waypoint: "))
    except ValueError:
        print("Invalid input. Exiting.")
        return

    # Define your waypoint coordinates
    waypoints = [
        (home_location.latitude + 0.0001, home_location.longitude, height),
        (home_location.latitude + 0.0001, home_location.longitude + 0.0001, height),
        (home_location.latitude, home_location.longitude + 0.0001, height),
    ]

    mission_items = []

    for wp in waypoints:
        mission_items.append(MissionItem(
            latitude_deg=wp[0],
            longitude_deg=wp[1],
            relative_altitude_m=wp[2],
            speed_m_s=5.0,
            is_fly_through=True,
            gimbal_pitch_deg=0.0,
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.NONE
        ))

    mission_plan = MissionPlan(mission_items)

    print("Uploading mission...")
    await drone.mission.upload_mission(mission_plan)

    print("Arming drone...")
    await drone.action.arm()

    print("Starting mission...")
    await drone.mission.start_mission()

    # Wait until mission is finished
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")
        if mission_progress.current == mission_progress.total:
            print("Mission complete!")
            break

    await asyncio.sleep(2)

    print("Returning to launch...")
    await drone.action.return_to_launch()

    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("Landed successfully.")
            break


async def failsafe_monitor(drone):
    async for status in drone.telemetry.status_text():
        if "Fail" in status.text or "Signal lost" in status.text:
            print(f"Failsafe triggered! Message: {status.text}")
            await drone.action.return_to_launch()
            break


async def main():
    drone = System()
    await asyncio.gather(
        drone_mission(drone),
        failsafe_monitor(drone)
    )

if __name__ == "__main__":
    asyncio.run(main())

