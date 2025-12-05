import os
import time
import streamlit as st
import httpx

# Prefer IPv4 loopback to avoid proxies/IPv6 resolution issues; allow override via env.
BACKEND = os.getenv("BACKEND_URL", "http://127.0.0.1:8000")

# Use a client that ignores system proxy settings to avoid local MITM/auto-proxy causing 502.
def _client():
    return httpx.Client(timeout=5.0, trust_env=False)

if "auto_poll" not in st.session_state:
    st.session_state["auto_poll"] = False

st.title("Dogwalker 控制面板")

st.markdown("前后端对齐 PRD 流程：选择模式+目标 → 领跑 → 掉队检测 → 激励 → 继续 → 到达/重置。")

status_box = st.empty()

def call_api(path: str, method: str = "get", json=None):
    with _client() as c:
        func = getattr(c, method)
        kwargs = {}
        if json is not None:
            kwargs["json"] = json
        r = func(f"{BACKEND}{path}", **kwargs)
        r.raise_for_status()
        return r.json()

def render_status():
    data = call_api("/status")
    status_box.info(f"模式: {data.get('mode')} | 阶段: {data.get('phase')} | 目标: {data.get('target')} | 提示: {data.get('message')}")
    status_box.json(data)

col_status1, col_status2 = st.columns(2)
with col_status1:
    if st.button("刷新状态"):
        try:
            render_status()
        except Exception as e:
            st.error(f"无法连接到后端: {e}")
with col_status2:
    st.session_state["auto_poll"] = st.checkbox("自动每2秒刷新状态", value=st.session_state["auto_poll"])

if st.session_state["auto_poll"]:
    try:
        render_status()
    except Exception as e:
        st.error(f"自动刷新失败: {e}")
    time.sleep(2)
    # 优先使用新版 rerun，其次 experimental_rerun，最后回退 meta refresh
    if hasattr(st, "rerun"):
        st.rerun()
    elif hasattr(st, "experimental_rerun"):
        st.experimental_rerun()
    else:
        st.markdown(
            "<meta http-equiv='refresh' content='2'>",
            unsafe_allow_html=True,
        )

st.markdown("---")

st.subheader("启动任务")
mode = st.selectbox("模式", ["health", "boss"])
target = st.text_input("目标点", value="茶水间" if mode == "health" else "导师办公室")
if st.button("开始领跑"):
    try:
        resp = call_api("/start", method="post", json={"mode": mode, "target": target})
        st.success("已开始领跑（后台会自动模拟掉队/激励/到达）")
        st.json(resp)
    except Exception as e:
        st.error(f"启动失败: {e}")

st.markdown("---")

st.subheader("行进中操作")
col1, col2, col3, col4, col5, col6 = st.columns(6)
with col1:
    if st.button("掉队检测"):
        try:
            st.json(call_api("/lag_detected", method="post"))
        except Exception as e:
            st.error(f"操作失败: {e}")
with col2:
    if st.button("语音激励"):
        try:
            st.json(call_api("/encourage", method="post"))
        except Exception as e:
            st.error(f"操作失败: {e}")
with col3:
    if st.button("继续领跑"):
        try:
            st.json(call_api("/resume", method="post"))
        except Exception as e:
            st.error(f"操作失败: {e}")
with col4:
    if st.button("到达目标"):
        try:
            st.json(call_api("/arrive", method="post"))
        except Exception as e:
            st.error(f"操作失败: {e}")
with col5:
    if st.button("急停"):
        try:
            st.json(call_api("/emergency_stop", method="post"))
        except Exception as e:
            st.error(f"操作失败: {e}")
with col6:
    if st.button("重置"):
        try:
            st.json(call_api("/reset", method="post"))
        except Exception as e:
            st.error(f"操作失败: {e}")
