from __future__ import annotations


def hold_current_pose(observation: dict) -> dict:
    state = observation["controller_state"]
    return {
        "target_tcp": state["tcp_pose"],
        "gripper_width": state["gripper_width"],
    }


def nudge_x(observation: dict) -> dict:
    state = observation["controller_state"]
    target = list(state["tcp_pose"])
    target[0] += 0.01
    return {
        "target_tcp": target,
        "gripper_width": state["gripper_width"],
    }

