"""Simple runtime state for the demo backend."""

STATE = {
    "mode": "idle",
    "task": None
}

def get_state():
    return STATE

def set_mode(mode: str):
    STATE["mode"] = mode

def set_task(task: str):
    STATE["task"] = task
