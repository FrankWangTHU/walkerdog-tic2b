from fastapi import FastAPI
from pydantic import BaseModel
from . import state
from .robot_controller import FakeDog

app = FastAPI(title="Dogwalker Backend")
robot = FakeDog()

class ModePayload(BaseModel):
    mode: str

@app.get("/status")
async def get_status():
    return state.get_state()

@app.post("/mode")
async def set_mode(payload: ModePayload):
    state.set_mode(payload.mode)
    return {"status": "ok", "mode": payload.mode}

@app.post("/simulate_walk")
async def simulate_walk():
    result = robot.walk()
    state.set_mode("walking")
    return {"result": result}
