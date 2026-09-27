# AGENTS.md

## Collaboration style (user preference)

- Be collaborative and conversational, not a monologue machine. Ask questions.
- **Check in on every guess.** Whenever I form a hypothesis or am about to assume
  something (about data, intent, or how something was run), state it and ask the user
  to confirm BEFORE acting on it. No silent guesses.
- When something is ambiguous (data interpretation, intent, root cause), ask the user
  instead of guessing or shipping an unverified conclusion.
- Never assume how the user ran something. If a measurement surprises you, ask them how
  they drove/recorded/launched it before drawing conclusions.
- Surface findings and open questions together, then let the user steer the next step.
- Prefer short back-and-forth over long unilateral action when the user is present.

## Project notes

- ROS2 Jazzy workspace at `ros2_ws/`; rover is a Raspberry Pi reachable over the LAN.
- Every ROS2 terminal must export `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`,
  `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`, or nodes silently fail to discover each other.
- Recorded bags live in `recordings/`, RTAB-Map databases in `maps/`.
- Keyboard teleop: `ros2 run rover_bridge rover_teleop` (publishes `/cmd_vel` at 20 Hz).
- **Never command low speeds.** Sub-stall commands stall the motors against floor
  friction, pull locked-rotor current and brown the rover out (observed shutdown).
  Minimum forward speed is **0.35 m/s**; rotations must also clear the stall band.
  See `docs/rover_low_speed_stall.md`.