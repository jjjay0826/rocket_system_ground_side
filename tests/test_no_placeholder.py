# -*- coding: utf-8 -*-
"""畫面上不可以出現 Qt Designer 的預留字（2026-08-01）。

ui_main.py 裡有 11 個 QLabel 的初值是 "TextLabel" —— Designer 產生的佔位符。
它們只在【有遙測時】才被覆寫，所以 backend 沒起來、或還沒收到資料的機器上，
畫面就一直掛著 "TextLabel"。兩台電腦看起來不一樣，其中一台就是卡在這個狀態。

順帶守一件更危險的事：health_* 那四個唯一的另一條設定路徑是
reset_gui_state()，而它把四個全部設成【綠色 OK】。/reset-data 之後畫面
宣稱 BMP/IMU/LoRa/SD 都健康，但那一刻一筆資料都還沒進來。
憑空的綠燈比 "TextLabel" 危險得多。
"""
import sys, re, pathlib
from _common import Checker, main_window, REPO

sys.path.insert(0, str(REPO))

# ui_main.py 裡預設成 "TextLabel" 的每一個
PLACEHOLDER_LABELS = [
    "gl_label", "health_lora", "health_imu", "health_sd", "health_bmp",
    "map_label", "chart_label_1", "chart_label_2", "chart_label_3",
    "version_label", "serial_label",
]


def run():
    c = Checker("畫面不得出現 TextLabel 預留字")

    # 先確認清單沒漏 —— .ui 改了要跟著改
    ui_src = (REPO / "src" / "gui" / "ui_main.py").read_text(encoding="utf-8")
    found = re.findall(r'self\.([a-z_0-9]+)\.setText\(_translate\("MainWindow", "TextLabel"\)\)',
                       ui_src)
    c.eq("★清單涵蓋 ui_main.py 裡所有的 TextLabel",
         sorted(set(found)), sorted(set(PLACEHOLDER_LABELS)),
         "ui 檔改了就要回來更新這份清單")

    w = main_window()

    # ── ① 剛開起來（還沒有任何遙測）就不該有 TextLabel ──────────────
    bad = [n for n in PLACEHOLDER_LABELS
           if getattr(w.ui, n).text().strip() == "TextLabel"]
    c.check("★啟動後畫面上沒有任何 TextLabel", not bad, f"仍是預留字：{bad}")

    for n in PLACEHOLDER_LABELS:
        t = getattr(w.ui, n).text()
        if n == "version_label":
            continue                     # 由既有邏輯填版號，空字串也可接受
        c.check(f"  {n} 有有意義的初值", bool(t.strip()), f"{n}={t!r}")

    # ── ② health_* 在沒有資料時必須是「未知」，不是綠色 OK ──────────
    for n in ("health_bmp", "health_imu", "health_lora", "health_sd"):
        t = getattr(w.ui, n).text()
        c.check(f"★{n} 初值不宣稱 OK", "OK" not in t, f"{n}={t!r}")

    # ── ③ ★reset 之後也不可以宣稱綠燈 ────────────────────────────
    # 這是本支最重要的一條：/reset-data 的語意是「沒有資料了」，
    # 不是「一切正常」。下一筆遙測到達時 update_ui 會覆寫成真實狀態。
    w.reset_gui_state()
    for n in ("health_bmp", "health_imu", "health_lora", "health_sd"):
        t = getattr(w.ui, n).text()
        c.check(f"★reset 後 {n} 不宣稱 OK", "OK" not in t,
                f"{n}={t!r} —— reset 那一刻一筆資料都還沒進來")
        qss = getattr(w.ui, n).styleSheet()
        c.check(f"  {n} 不是綠底", "150, 200, 150" not in qss, qss[:60])

    # ── ④ 收到真實遙測後要能變回 OK ──────────────────────────────
    from src.core.models import SensorData
    from _common import frame
    w.update_ui(SensorData.from_new_format(frame(t=1, sq=1, mod="F")))
    ok = [n for n in ("health_bmp", "health_imu", "health_lora", "health_sd")
          if "OK" in getattr(w.ui, n).text()]
    c.eq("★有真實遙測（MOD:F）後四個都變 OK", len(ok), 4,
         f"實際 {ok} —— 覆寫路徑不能因為改初值而壞掉")

    # ── ⑤ 字型有後備（兩台機器版面不一致的來源之一）────────────────
    c.check("★等寬字型有列後備", "setFamilies(" in ui_src,
            "只指定 0xProto Nerd Font Mono 的話，沒裝的機器會由 Qt 任意替換，"
            "字寬行高跟著變")

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
