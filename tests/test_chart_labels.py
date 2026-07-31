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

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
