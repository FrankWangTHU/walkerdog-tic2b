import streamlit as st
import httpx

BACKEND = "http://localhost:8000"

st.title("Dogwalker 控制面板")

if st.button("刷新状态"):
    try:
        r = httpx.get(f"{BACKEND}/status", timeout=2.0)
        r.raise_for_status()
        st.json(r.json())
    except Exception as e:
        st.error(f"无法连接到后端: {e}")

st.markdown("---")

if st.button("模拟遛狗（simulate_walk）"):
    try:
        r = httpx.post(f"{BACKEND}/simulate_walk", timeout=5.0)
        r.raise_for_status()
        st.success("已触发遛狗任务")
        st.json(r.json())
    except Exception as e:
        st.error(f"操作失败: {e}")

st.markdown("---")

new_mode = st.selectbox("切换模式", ["idle", "walking", "paused"])
if st.button("设置模式"):
    try:
        r = httpx.post(f"{BACKEND}/mode", json={"mode": new_mode}, timeout=2.0)
        r.raise_for_status()
        st.success(f"模式已设置为 {new_mode}")
    except Exception as e:
        st.error(f"设置失败: {e}")
