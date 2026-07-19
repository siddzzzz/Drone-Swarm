import asyncio
import json
import websockets
import numpy as np
from core.drone import Drone
from core.path_planner import PathPlanner

class SimulatorServer:
    def __init__(self, host="127.0.0.1", port=8765):
        self.host = host
        self.port = port
        self.clients = set()
        
        # Simulation Settings
        self.current_step = 1  # 1 to 5
        self.num_drones = 10  # Default
        self.path_type = "circle"  # circle, figure_eight, helix
        
        # State variables
        self.drones = []
        self.cached_paths = []
        self.loop_hz = 60
        self.is_running = True
        
        # PID default gains (xy=translation, z=altitude)
        self.kp_xy = 3.5
        self.ki_xy = 0.15
        self.kd_xy = 2.2
        self.kp_z = 3.0
        self.ki_z = 0.05
        self.kd_z = 3.2
        
        self.reset_simulation()
        
    def reset_simulation(self):
        """Re-initializes drones and sets independent local trajectory planning atomically."""
        # 1. Ask path planner for sparse waypoint mission configurations for the entire swarm (single pass)
        swarm_missions = PathPlanner.plan_for_swarm(
            num_drones=self.num_drones,
            path_type=self.path_type,
            base_height=5.0
        )
        
        # 2. Instantiate individual independent drones and set their waypoints in a temporary list
        temp_drones = []
        for i in range(self.num_drones):
            waypoints, times, loop, colors = swarm_missions[i]
            start_pos = waypoints[0] if len(waypoints) > 0 else [0.0, 0.0, 0.0]
            drone = Drone(drone_id=i, initial_pos=start_pos)
            drone.set_mission_waypoints(waypoints, times, loop=loop, colors=colors)
            
            # Apply dynamic/kinematic state based on active step
            drone.is_kinematic = (self.current_step == 1)
            # Apply server's current PID gains
            drone.set_pid_gains(self.kp_xy, self.ki_xy, self.kd_xy, self.kp_z, self.ki_z, self.kd_z)
            
            temp_drones.append(drone)
            
        # 3. Pre-compute and cache reference paths for GCS path lines (only once per shape reset!)
        # We sample each drone's local spline solver at 50 points. Done for all steps to show reference overlay.
        temp_paths = []
        for drone in temp_drones:
            if len(drone.waypoints) > 1:
                times = np.linspace(drone.waypoint_times[0], drone.waypoint_times[-1], 50)
                sample_path = [drone.evaluate_trajectory(t).tolist() for t in times]
                temp_paths.append(sample_path)
                    
        # Atomic assignments to prevent concurrency race conditions in simulation_loop
        self.cached_paths = temp_paths
        self.drones = temp_drones
        print(f"Simulation reset: {self.num_drones} independent drones initialized on {self.path_type} mission (Step {self.current_step}).")

    async def register(self, websocket):
        self.clients.add(websocket)
        print(f"Client connected. Total clients: {len(self.clients)}")
        await websocket.send(json.dumps({
            "type": "config",
            "step": self.current_step,
            "num_drones": self.num_drones,
            "path_type": self.path_type
        }))

    async def unregister(self, websocket):
        self.clients.remove(websocket)
        print(f"Client disconnected. Total clients: {len(self.clients)}")

    async def handle_message(self, websocket, message):
        try:
            data = json.loads(message)
            msg_type = data.get("type")
            
            if msg_type == "set_step":
                self.current_step = int(data.get("value"))
                self.reset_simulation()
                
            elif msg_type == "set_drones":
                self.num_drones = int(data.get("value"))
                self.reset_simulation()
                
            elif msg_type == "set_path_type":
                self.path_type = data.get("value")
                self.reset_simulation()
                
            elif msg_type == "set_pid":
                self.kp_xy = float(data.get("kp_xy", 3.5))
                self.ki_xy = float(data.get("ki_xy", 0.15))
                self.kd_xy = float(data.get("kd_xy", 2.2))
                self.kp_z = float(data.get("kp_z", 4.5))
                self.ki_z = float(data.get("ki_z", 0.2))
                self.kd_z = float(data.get("kd_z", 2.8))
                
                # Apply live gains to all active drones
                for drone in self.drones:
                    drone.set_pid_gains(self.kp_xy, self.ki_xy, self.kd_xy, self.kp_z, self.ki_z, self.kd_z)
                print(f"GCS updated PID gains: XY=[{self.kp_xy}, {self.ki_xy}, {self.kd_xy}] Z=[{self.kp_z}, {self.ki_z}, {self.kd_z}]")
                
        except Exception as e:
            print(f"Error handling message: {e}")

    async def broadcast(self, message):
        if not self.clients:
            return
        await asyncio.gather(*[client.send(message) for client in self.clients])

    async def simulation_loop(self):
        """Core simulator loop running at 60Hz."""
        dt = 1.0 / self.loop_hz
        while self.is_running:
            start_time = asyncio.get_event_loop().time()
            
            # Step 1: Tell each drone to update its state based on elapsed time dt
            # Each drone independently evaluates its local spline trajectory and moves.
            for drone in self.drones:
                drone.update(dt)
                
            telemetry = {
                "type": "telemetry",
                "step": self.current_step,
                "drones": [drone.to_dict() for drone in self.drones],
                "paths": self.cached_paths
            }
            
            await self.broadcast(json.dumps(telemetry))
            
            elapsed = asyncio.get_event_loop().time() - start_time
            sleep_time = max(0, dt - elapsed)
            await asyncio.sleep(sleep_time)

    async def socket_handler(self, websocket):
        await self.register(websocket)
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            await self.unregister(websocket)

    async def run(self):
        asyncio.create_task(self.simulation_loop())
        print(f"Starting WebSocket server on ws://{self.host}:{self.port}...")
        async with websockets.serve(self.socket_handler, self.host, self.port):
            await asyncio.Future()

if __name__ == "__main__":
    server = SimulatorServer()
    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        print("\nStopping Simulator Server.")
