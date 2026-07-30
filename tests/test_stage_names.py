# -*- coding: utf-8 -*-
"""階段顯示必須與火箭端的 FlightState_t 一致。

這支存在的理由：2026-07-30 之前 GUI 用的是一份 12 狀態清單，而火箭只有 5 個。
後果不是名字難看而已 ——

    火箭送 ST:2（正在放傘）→ 螢幕顯示「IGNITION」、圖表打綠色 IGNITION 標記
    火箭送 ST:4（已落地）  → 螢幕顯示「BURNOUT」，後面 7 格永遠灰著
    T0 錨在 index 2        → 所有 T+x.xxs 都是從「開傘那一刻」起算

文件（doc/telemetry_format.md）先被改對了，但同一份錯誤表也寫在執行中的
程式碼裡，沒被抓到。所以這裡直接跟韌體的列舉比。
"""
import sys, re
from datetime import datetime, timedelta
from _common import Checker, firmware_repo, REPO

sys.path.insert(0, str(REPO))


def run():
    c = Checker("階段名稱與火箭端 FlightState_t 一致")
    from src.gui.visualizers.stage_display import (
        ROCKET_STAGES, EVENT_STAGES, T0_STAGE, StageDisplayer)

    # ── 與韌體列舉逐字比對 ──
    fw = firmware_repo()
    if fw is None:
        c.skip("找不到韌體 repo，改用硬編碼期望值")
        expect = ["IDLE", "LAUNCHED", "DEPLOYING", "DEPLOYED", "LANDED"]
    else:
        src = (fw / "firmware-rocket" / "Core" / "Src" / "main.c").read_text(
            encoding="utf-8", errors="ignore")
        m = re.search(r"typedef enum \{(.*?)\} FlightState_t;", src, re.S)
        expect = [n.replace("FLIGHT_", "")
                  for n in re.findall(r"(FLIGHT_[A-Z]+)", m.group(1))]
    c.eq("階段清單與 FlightState_t 完全相同", list(ROCKET_STAGES), expect)
    c.eq("只有 5 個狀態（不是 12 個）", len(ROCKET_STAGES), 5)
    c.check("★ index 2 是 DEPLOYING 不是 IGNITION",
            ROCKET_STAGES[2] == "DEPLOYING",
            "火箭看不到點火 —— 它最早知道的是離架後 2.5g×200ms")

    # ── 事件對照表不得指向不存在的狀態 ──
    bad = [k for k in EVENT_STAGES if k >= len(ROCKET_STAGES)]
    c.eq("事件表沒有指向不存在的狀態", bad, [])
    c.check("開傘事件掛在 ST:2", EVENT_STAGES.get(2, ("",))[0] == "PARACHUTE_DEPLOY",
            str(EVENT_STAGES.get(2)))

    # ── T0 必須是離架，不是開傘 ──
    c.eq("T0 錨定在 LAUNCHED", T0_STAGE, 1)
    c.eq("T0 對應的名稱", ROCKET_STAGES[T0_STAGE], "LAUNCHED")

    # ── 實跑一遍完整飛行，確認時間軸與越界處理 ──
    from PyQt6.QtWidgets import QListWidget
    from _common import qt_app
    qt_app()
    sd = StageDisplayer(QListWidget())
    t0 = datetime(2026, 7, 30, 10, 0, 0)
    for st, dt in ((0, 0), (1, 1.3), (2, 18.0), (3, 19.0), (4, 220.0)):
        sd.update(st, t0 + timedelta(seconds=dt))
    c.eq("走完 0→4 目前停在 LANDED", sd.current_stage, 4)
    c.eq("五個狀態都造訪過", sorted(sd.visited_stages), [0, 1, 2, 3, 4])
    rel = (sd.stage_times[2] - sd.stage_times[T0_STAGE]).total_seconds()
    c.check("開傘時間相對離架而非相對開傘", abs(rel - 16.7) < 1e-6,
            f"T+{rel:.2f}s（若 T0 錯錨在開傘，這裡會是 0.00）")

    sd.reset()
    sd.update(9, t0)          # 越界：韌體真的改成 12 態時的行為
    c.check("越界的 ST 不會 IndexError，也不會畫出錯誤名稱",
            sd.current_stage == -1, "舊碼會直接 self.stages[9] 爆掉")

    # ── 推導事件與火箭回報要分得開 ──
    sd.reset()
    sd.update(1, t0)
    c.check("推導事件可加入", sd.mark_derived("BURNOUT", t0 + timedelta(seconds=5.9)))
    c.check("同名推導事件不重複", not sd.mark_derived("BURNOUT", t0))
    c.eq("推導事件不混進火箭狀態清單", len(sd.stages), 5)
    labels = sd._labels()
    c.check("推導事件標示為「地面推導」",
            any("地面推導" in x for x in labels),
            "免得日後有人把推導值當成火箭實測回報")
    sd.reset()
    c.eq("reset 清掉推導事件", sd.derived, [])

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
