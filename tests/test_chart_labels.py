# -*- coding: utf-8 -*-
"""圖表上的事件縮寫（2026-08-01）。

發現經過：回放時圖上出現一條綠色虛線，標籤是「✅ ch」。查下去 ——
_ABBR_MAP 是一張 10 筆的【完全比對】字典，鍵是 "[CMD] ARM" 這種完整字串。
但事件標籤現在有 23 個產生點，大多是帶 emoji 與頻道名的 f-string：

    [✅ ch1 DPL OK]     [🚫 ch2 REJECT]     [🔴 ch1 基準漂移 12m]

那些永遠對不中，全部掉進後備的「取 [] 內前 4 個字元」，於是圖上一整排
都是「emoji + ch」—— 彼此分不出來，也看不出是哪一塊板。

這支的重點不是「縮寫好不好看」，是【兩個不同事件不可以縮成同一個字串】。
"""
import sys
from _common import Checker, main_window

sys.path.insert(0, str(__import__("_common").REPO))

# (事件標籤, 期望縮寫)。左邊這些都是 main_window 裡真的會送出去的。
CASES = [
    ("[CMD] ARM",                    "ARM"),
    ("[CMD] DPL",                    "DPL"),
    ("[CMD] CAL",                    "CAL"),
    ("[CMD] Reset Angle",            "RST"),
    ("[CMD] SETCH 72",               "CH72"),
    ("[BURNOUT]",                    "BRN"),
    ("[APOGEE 903m]",                "APG"),
    ("[LAUNCH]",                     "LNCH"),
    ("[DESCENT]",                    "DESC"),
    ("[LANDED]",                     "LAND"),
    ("[PARACHUTE_DEPLOY]",           "DPL"),
    ("[🧪 GNDTEST]",                 "GND+"),
    ("[🧪 GNDTEST OFF]",             "GND-"),
    ("[🧭 軸向 -x]",                  "AXIS"),
    ("[🛑 上升中，再按一次確認]",       "!ASC"),
    # ★ 帶頻道的：縮寫必須帶上頻道號
    ("[🔓 ch1 ARMED]",               "ARM1"),
    ("[🔓 ch2 ARMED]",               "ARM2"),
    ("[🚫 ch1 REJECT]",              "REJ1"),
    ("[🚫 ch2 REJECT]",              "REJ2"),
    ("[✅ ch1 DPL OK]",              "DPL✓1"),
    ("[✅ ch2 ARM OK]",              "ARM✓2"),
    ("[🔴 ch1 保險絲熔斷]",           "FUSE1"),
    ("[🔓 ch2 已武裝]",               "ARMD2"),
    ("[🔴 ch1 基準漂移 12m]",         "DRIFT1"),
    ("[🟠 ch2 基準漂移 7m]",          "DRIFT2"),
    ("[🔴 ch1 DPL 未確認]",           "!ACK1"),
]


def run():
    c = Checker("圖表事件縮寫")
    w = main_window()

    for label, want in CASES:
        got = w._chart_abbr(label)
        c.eq(f"{label}", got, want)

    # ── ★真正要守的：不同事件不可以縮成同一個字串 ──────────────────
    seen = {}
    dup = []
    for label, _ in CASES:
        a = w._chart_abbr(label)
        if a in seen and seen[a] != label:
            dup.append((a, seen[a], label))
        seen[a] = label
    # DPL 有兩個來源（遠端指令 / 火箭回報開傘）縮成同一個是刻意的
    dup = [d for d in dup if d[0] != "DPL"]
    c.check("★不同事件不會縮成同一個縮寫", not dup,
            "、".join(f"{a}: {x} vs {y}" for a, x, y in dup))

    # ── ch1 與 ch2 一定要分得出來 ──────────────────────────────────
    for base in ("[🔓 {} ARMED]", "[🚫 {} REJECT]", "[✅ {} DPL OK]",
                 "[🔴 {} 基準漂移 12m]"):
        a1 = w._chart_abbr(base.format("ch1"))
        a2 = w._chart_abbr(base.format("ch2"))
        c.check(f"★{base.format('chN')} 分得出頻道", a1 != a2, f"{a1} vs {a2}")

    # ── 長度要能放進圖裡 ──────────────────────────────────────────
    longest = max((w._chart_abbr(l) for l, _ in CASES), key=len)
    c.check("縮寫夠短（≤6 字）", len(longest) <= 6, f"最長 {longest!r}")

    # ── 舊缺陷的直接回歸：不可以再出現裸截斷 ──────────────────────
    for label in ("[✅ ch1 DPL OK]", "[🔓 ch1 ARMED]", "[🚫 ch1 REJECT]"):
        got = w._chart_abbr(label)
        c.check(f"★{label} 不再縮成「emoji + ch」",
                got != "✅ ch" and not got.startswith(("✅", "🔓", "🚫")),
                f"實際 {got!r}")

    # ── 認不出來的標籤也要給出可讀的東西，不能是空字串 ──────────────
    for odd in ("[某個沒見過的事件]", "沒有中括號", "[]"):
        got = w._chart_abbr(odd)
        c.check(f"未知標籤 {odd!r} → {got!r}（非空）", bool(got))

    # ── ★事件標記必須畫在【觸發它的那一幀】，不是上一幀 ────────────
    # update_ui 是在最後才 self.latest_data = data，而所有推導都跑在那之前。
    # broadcast_event 若用 self.latest_data，每個標記就會早整整一個遙測
    # 週期（2Hz → 0.5s）。畫面上：APOGEE 標在高度峰值左邊 0.5 秒，
    # 開傘標記反而落在峰值上。事後論證開傘時序時每個事件系統性早半秒，
    # 而 C 備援最小餘裕只有 1.69 秒。
    import inspect
    from src.core.models import SensorData
    from _common import frame

    sig = inspect.signature(w.broadcast_event)
    c.check("★broadcast_event 收得下觸發幀（src_data）",
            "src_data" in sig.parameters,
            "沒有的話只能拿 self.latest_data —— 那是上一幀")

    src = inspect.getsource(w.update_ui)
    c.check("★推導事件有把觸發幀傳進去",
            'self.broadcast_event(f"[{name}]", color, data)' in src,
            "derive() 不傳 data 的話，標記會落在上一幀")
    c.check("★狀態轉換事件也傳觸發幀",
            "self.broadcast_event(f\"[{ev_name}]\", ev_color, data)" in src)

    # 實測 x 位置：兩幀相差 0.5s，標記必須落在後面那一幀
    w.reset_gui_state()
    w.start_time = 0.0
    prev = SensorData.from_new_format(frame(t=1000, sq=1))
    prev.gs_timestamp = 100.0
    cur = SensorData.from_new_format(frame(t=1500, sq=2))
    cur.gs_timestamp = 100.5
    w.latest_data = prev

    got_x = []
    real_add = w.chart_1.add_event_marker
    w.chart_1.add_event_marker = lambda x, *a, **k: got_x.append(x)
    try:
        w.broadcast_event("[BURNOUT]", "#FF9100", cur)      # 傳觸發幀
        w.broadcast_event("[BURNOUT2]", "#FF9100")          # 不傳 → 用 latest
    finally:
        w.chart_1.add_event_marker = real_add
    c.eq("★傳了觸發幀 → 標在該幀（100.5）", round(got_x[0], 3), 100.5)
    c.eq("沒傳 → 沿用 latest_data（100.0），指令事件用這條", round(got_x[1], 3), 100.0)
    c.check("★兩者差一個遙測週期，就是原本的偏差量",
            abs((got_x[0] - got_x[1]) - 0.5) < 1e-9)

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
