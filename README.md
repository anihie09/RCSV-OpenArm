# RCSV-OpenArm

TASK 1 : CAN Interface Setup

I followed the setup procedure at docs.openarm.dev/tutorial/setup
The script is named as scripts/setup_can.sh. This was done with no CAN hardware and interfaces and no access to
ppa:openarm/main repo. I also tried to bring up virtual CAN but the vcan module isn't available.

scripts/mock_can_cli_py tries the real vcan0/vcan1 path so it could give me ip link show output using any machine.
However, since this isn't a live capture, I opted to print the exact documented command sequence with the expected output. That output is in scripts/tasks1_output.txt. 
If I were using real hardware, running setup_can.sh hopefully gives me the live output of can0 and can1 showing UP,
LOWER_UP in ip link show. There should also be confirmation that the 0 position was set for both 7 DOF arms. 


TASK 2 : CAN Data Reading

can/can_reader.py defines a shared interface so the rest of the pipeline (sync, storage, dashboard — not built in this submission, see Task 3-5 notes below) doesn't care whether data comes from hardware or simulation:

- JointState — a dataclass holding timestamp, position, velocity, torque (each a 7-element list for OpenArm 2.0's 7 DOF), and arm_id (can0/can1).

- CANReader — abstract interface: start(), stop(), read_joint_state() (pull), register_callback() (push).

- LiveCANReader — the real implementation, written directly against the documented openarm_can Python bindings (OpenArm, MotorType, CallbackMode, init_arm_motors, set_callback_mode_all, refresh_all / recv_all, get_position()/get_velocity()/get_torque()), following the "Real-time Monitoring" pattern from docs.openarm.dev/api-reference/can. This is the code that would run on the robot; it raises a clear RuntimeError if openarm_can isn't installed, rather than failing silently.

- MockCANReader — a software simulator implementing the exact same interface. It generates smooth per-joint sinusoidal position/velocity trajectories (different amplitude/frequency/phase per joint) plus a simple gravity-proportional torque model, polled at a configurable rate (default 1 kHz, matching the docs' guidance on 1 kHz control-loop limits for 8-motor setups).

Running python3 can/can_reader.py produces a live stream of (t, position, velocity, torque) for joint 0, confirming the interface works end-to-end:

t=68.009 pos[0]=+0.026 vel[0]=+0.282 tau[0]=+0.035
t=68.111 pos[0]=+0.054 vel[0]=+0.278 tau[0]=+0.057
...

DESIGN DECISIONS

- Shared interface, swappable backend. MockCANReader and LiveCANReader are interchangeable so swapping one line in main.py (not built here) is the only change needed when hardware becomes available. This was the decision for a take home with no hardware access because it means none of the sync/storage/dashboard logic would need to change later.

- Push and pull both supported. register_callback() lets the multi-camera sync layer (Task 3) are related to new joint states as they arrive, while read_joint_state() lets the dashboard (Task 5) poll on its own refresh cycle without blocking.

- Per-bus readers. OpenArm 2.0 is bimanual (can0/can1), so each CANReader instance is scoped to one bus/arm; a coordinator would run two readers in parallel.

- Callback mode discipline. The LiveCANReader explicitly sets CallbackMode.STATE before polling, per the docs' warning that callback mode must match the type of frame being received (STATE for control/state, PARAM for parameter queries).

- Timeout choices. recv_all(300) (300 µs) is used in the live reader's poll loop, matching the docs' recommendation for fast control-cycle operations; recv_all(2000) is used during the one time enable_all() / init sequence, matching the guidance for slower setup operations.

KEY TRADE OFFS : 

Sinusoidal mock trajectories vs. recorded real demo data 
-> Simple and easier to verify by eye but doesn't capture real contact dynamics, sensor noise characteristics or torque spikes from a real teleop session.

Polling thread per reader vs. asyncio
-> Threads are simpler, and SocketCAN's blocking recv_all maps naturally to a dedicated thread per bus. Asyncio would scale better with many concurrent I/O sources (cameras + 2 CAN buses + REST API), which matters more once Tasks 3-5 are built.

Raising on missing openarm_can vs. silently falling back to mock
-> Made the live/mock boundary explicit and loud, so a partially configured machine doesn't silently record fake data thinking it's real.


WHAT I'D DO NEXT : 

- Task 3 (camera sync): Tag every JointState and camera frame with a shared monotonic timestamp at capture time; run cameras at their native rates (wrist Arducams and ceiling likely 30-60 Hz, ZED at its own rate) and resample/interpolate joint state to each frame's timestamp rather than trying to force all sensors to a common clock. For the 1 kHz joint stream, the nearest or linearly-interpolated joint state at each frame timestamp is enough, given the difference in sensor bandwidths.

- Task 4 (storage): HDF5 per episode -> one dataset per joint signal plus one dataset per camera (compressed video or chunked image arrays), with episode-level JSON metadata (duration, task label, success flag). HDF5 is well-supported in the robot learning ecosystem (LeRobot, RLDS converters) and supports partial/streaming reads, which matters for a REST API that serves individual episodes without loading the whole file.

- Task 5 (dashboard): A small FastAPI + WebSocket backend streaming JointState and downsampled camera frames to a lightweight React/Plotly frontend, with Start/Stop wired to the episode writer from Task 4.

NOTE : 
The CAN reader abstraction built here (CANReader, JointState) is designed to plug directly into all three of the above without modification. AI use was included in this take home assignment. 
