"""探针：在 Isaac Sim 6.0.1 kit 上下文中实测 leisaac 需要的 isaacsim.core 模块路径是否可用。"""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

mods = [
    "isaacsim.core.experimental.utils.prim",
    "isaacsim.core.experimental.utils",
    "isaacsim.core.utils.prims",
    "isaacsim.core.simulation_manager",
    "isaacsim.core.prims",
]
for m in mods:
    try:
        __import__(m, fromlist=["*"])
        print(f"[probe] OK   {m}")
    except Exception as e:  # noqa: BLE001
        print(f"[probe] FAIL {m}: {type(e).__name__}: {e}")

try:
    import isaacsim.core.experimental.utils.prim as p

    print("[probe] get_prim_at_path available:", hasattr(p, "get_prim_at_path"))
except Exception as e:  # noqa: BLE001
    print("[probe] get_prim_at_path probe failed:", e)

app.close()
