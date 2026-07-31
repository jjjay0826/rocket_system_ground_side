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
    # ★2026-08-01：畫面改回完整飛行序列（11 列），T0_STAGE 現在是【序列列號】
    #   而不是 ST 值。所以驗的是「那一列真的叫 LAUNCHED」，不是它等於 1。
    from src.gui.visualizers.stage_display import FLIGHT_SEQUENCE, ST_ROW
    c.eq("T0 對應的名稱", FLIGHT_SEQUENCE[T0_STAGE][0], "LAUNCHED")
    c.eq("T0 就是 ST:1 對應的那一列", T0_STAGE, ST_ROW[1])

    # ── 完整飛行序列：火箭列與推導列 ────────────────────────────────
    rkt = [n for n, _, s in FLIGHT_SEQUENCE if s == "rkt"]
    gnd = [n for n, _, s in FLIGHT_SEQUENCE if s == "gnd"]
    c.eq("★序列裡的火箭列＝FlightState_t 逐字相同", rkt, expect,
         "畫面寫 PARACHUTE_DEPLOY、原始碼叫 DEPLOYING，事後對照的人會 grep 不到")
    c.check("★每個 ST 值都有對應的列，且不重複",
            sorted(ST_ROW) == [0, 1, 2, 3, 4]
            and len(set(ST_ROW.values())) == 5,
            f"{ST_ROW}")
    c.check("★ST_ROW 的對應列真的是火箭列",
            all(FLIGHT_SEQUENCE[r][2] == "rkt" for r in ST_ROW.values()))
    c.check("★ST 值遞增，對應的列號也遞增（時間軸不會倒退）",
            [ST_ROW[k] for k in sorted(ST_ROW)] == sorted(ST_ROW.values()))
    c.check("推導列都在火箭列之間，不是附在最後",
            gnd and FLIGHT_SEQUENCE[0][2] == "rkt"
            and FLIGHT_SEQUENCE[-1][2] == "rkt",
            f"推導列 {gnd}")
    c.check("氣囊已從序列移除（2026-07-31）",
            not any("AIRBAG" in n for n, _, _ in FLIGHT_SEQUENCE))

    # ── 實跑一遍完整飛行，確認時間軸與越界處理 ──
    from PyQt6.QtWidgets import QListWidget
    from _common import qt_app
    qt_app()
    sd = StageDisplayer(QListWidget())
    t0 = datetime(2026, 7, 30, 10, 0, 0)
    for st, dt in ((0, 0), (1, 1.3), (2, 18.0), (3, 19.0), (4, 220.0)):
        sd.update(st, t0 + timedelta(seconds=dt))
    c.eq("走完 0→4 目前停在 LANDED", sd.current_row, ST_ROW[4])
    c.eq("五個火箭狀態都造訪過", sorted(sd.visited_rows),
         sorted(ST_ROW.values()))
    rel = (sd.row_times[ST_ROW[2]] - sd.row_times[T0_STAGE]).total_seconds()
    c.check("開傘時間相對離架而非相對開傘", abs(rel - 16.7) < 1e-6,
            f"T+{rel:.2f}s（若 T0 錯錨在開傘，這裡會是 0.00）")
    c.eq("★畫面列數＝完整飛行序列（不是只有 5 列）",
         len(sd._labels()), len(FLIGHT_SEQUENCE))

    sd.reset()
    sd.update(9, t0)          # 越界：韌體真的改成 12 態時的行為
    c.check("越界的 ST 不會 IndexError，也不會畫出錯誤名稱",
            sd.current_row == -1, "舊碼會直接 self.stages[9] 爆掉")

    # ── 推導事件要點亮序列裡對應的那一列 ──
    sd.reset()
    sd.update(1, t0)
    c.check("推導事件可加入", sd.mark_derived("BURNOUT", t0 + timedelta(seconds=5.9)))
    c.check("同名推導事件不重複", not sd.mark_derived("BURNOUT", t0))
    burn_row = next(i for i, (n, _, _) in enumerate(FLIGHT_SEQUENCE) if n == "BURNOUT")
    c.check("★BURNOUT 點亮的是序列裡 BURNOUT 那一列", burn_row in sd.derived_rows,
            f"derived_rows={sd.derived_rows}")
    c.check("★帶後綴也對得上（APOGEE 1040m → APOGEE 列）",
            sd.mark_derived("APOGEE 1040m", t0 + timedelta(seconds=16))
            and next(i for i, (n, _, _) in enumerate(FLIGHT_SEQUENCE)
                     if n == "APOGEE") in sd.derived_rows)
    labels = sd._labels()
    c.check("推導列標示為「推導」，未推導的標「—」",
            any("推導" in x for x in labels) and any("—" in x for x in labels),
            "免得日後有人把推導值當成火箭實測回報")
    c.check("序列外的推導事件仍可附在最後",
            sd.mark_derived("SOMETHING_ELSE", t0) and len(sd.extra) == 1)
    sd.reset()
    c.check("reset 清掉推導事件與時間軸",
            not sd.derived_rows and not sd.extra and not sd.row_times
            and not sd.visited_rows and sd.current_row == -1)
    c.eq("reset 之後畫面仍是完整序列（不是空的）",
         len(sd._labels()), len(FLIGHT_SEQUENCE))

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
