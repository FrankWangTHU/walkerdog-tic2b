import asyncio
from fastapi import FastAPI
from pydantic import BaseModel
from . import state

app = FastAPI(title="Dogwalker Backend")

# background mock flow task
mock_task: asyncio.Task | None = None

class StartPayload(BaseModel):
    mode: str
    target: str

async def mock_navigation_flow():
    """Simulate PRD flow without hardware/ROS."""
    steps = [
        ("leading", "领跑中，保持跟随。", 2),
        ("lag_detected", "检测到掉队，停车并转身等待。", 2),
        ("encourage", "别怂，我在你前面！", 2),
        ("leading", "重新领跑中。", 2),
        ("arrived", "已送达目标，待命。", 0),
    ]
    for phase, message, delay in steps:
        state.set_phase(phase, message)
        if state.should_stop_flow():
            break
        if delay > 0:
            try:
                await asyncio.sleep(delay)
            except asyncio.CancelledError:
                break

def stop_mock_task():
    global mock_task
    if mock_task and not mock_task.done():
        mock_task.cancel()
    mock_task = None

@app.get("/status")
async def get_status():
    return state.snapshot()

@app.post("/start")
async def start_task(payload: StartPayload):
    stop_mock_task()
    updated = state.start_task(payload.mode, payload.target)
    # fire-and-forget mock flow
    global mock_task
    mock_task = asyncio.create_task(mock_navigation_flow())
    return {"status": "ok", "state": updated}

@app.post("/lag_detected")
async def lag_detected():
    stop_mock_task()
    updated = state.set_phase("lag_detected", "检测到掉队，停车并转身等待。")
    return {"status": "ok", "state": updated}

@app.post("/encourage")
async def encourage():
    stop_mock_task()
    updated = state.set_phase("encourage", "别怂，我在你前面！")
    return {"status": "ok", "state": updated}

@app.post("/resume")
async def resume_leading():
    stop_mock_task()
    updated = state.set_phase("leading", "重新领跑中。")
    return {"status": "ok", "state": updated}

@app.post("/arrive")
async def arrive():
    stop_mock_task()
    updated = state.set_phase("arrived", "已送达目标，待命。")
    return {"status": "ok", "state": updated}

@app.post("/emergency_stop")
async def emergency_stop():
    stop_mock_task()
    updated = state.emergency_stop()
    return {"status": "ok", "state": updated}

@app.post("/reset")
async def reset():
    stop_mock_task()
    updated = state.reset()
    return {"status": "ok", "state": updated}
