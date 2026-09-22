#!/usr/bin/env python
"""启动空的 Isaac Sim GUI 窗口。关闭窗口即退出。"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": False})
while app.is_running():
    app.update()
app.close()
