"""Simple runtime state for the demo backend with a lightweight state machine."""

STATE = {
    "mode": "idle",       # idle | health | boss
    "target": None,       # e.g., 茶水间 / 导师办公室
    "phase": "idle",      # idle | leading | lag_detected | encourage | arrived | emergency_stop
    "message": "待命",    # human-friendly status text
    "voice_hint": ""      # optional voice text to play/speak
}

def snapshot():
    return STATE.copy()

def start_task(mode: str, target: str):
    STATE["mode"] = mode
    STATE["target"] = target
    STATE["phase"] = "leading"
    STATE["message"] = f"领跑中 → 目标: {target}"
    STATE["voice_hint"] = "出发，跟上我～" if mode == "health" else "战备模式，别怂，跟上我！"
    return snapshot()

def set_phase(phase: str, message: str = ""):
    STATE["phase"] = phase
    if message:
        STATE["message"] = message
        STATE["voice_hint"] = message
    return snapshot()

def reset():
    STATE["mode"] = "idle"
    STATE["target"] = None
    STATE["phase"] = "idle"
    STATE["message"] = "待命"
    STATE["voice_hint"] = ""
    return snapshot()

def emergency_stop(message: str = "急停，等待人工干预"):
    STATE["phase"] = "emergency_stop"
    STATE["message"] = message
    STATE["voice_hint"] = message
    return snapshot()

def should_stop_flow() -> bool:
    return STATE["phase"] in ["emergency_stop", "idle"]
