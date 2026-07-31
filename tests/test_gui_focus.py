# -*- coding: utf-8 -*-
"""切換焦點頻道不得摧毀資料；火工品按鈕的寬度與兩段式保險。

原缺陷：set_focus_channel() 直接呼叫 reset_gui_state()，來回切一次
兩個頻道的圖表、統計、地圖就全沒了。雙板熱備援的重點就是隨時能比對兩塊板，
切過去看一眼再切回來資料就消失，等於這個功能不能用。
"""
import sys, time
from datetime import datetime
from _common import Checker, main_window, REPO

sys.path.insert(0, str(REPO))


def run():
    c = Checker("焦點切換保留資料 / 按鈕寬度 / 兩段式保險")
    from PyQt6.QtWidgets import QPushButton
    from src.core.models import SensorData

    w = main_window()
    w.reset_gui_state()

    def mk(stage=0, alt=100.0):
        d = SensorData(rotationRoll=0.0, rotationPitch=0.0, direction=0.0,
                       timestamp=datetime.now(), stage=stage, failedTasks=[],
                       location=(0.0, 0.0))
        d.gs_timestamp = time.time()
        d.kfh_height = alt
        d.vz = -5.0
        return d

    # ── ch1 累積資料 ──
    for i in range(20):
        w.update_ui_from_zmq("ch1", mk(alt=100.0 + i * 10))
    ov_before = len(w.alt_overlays["ch1"]["kh"]._x)
    max_h_ch1 = w.max_height
    c.check("ch1 統計有累積", max_h_ch1 > 0, f"max_height={max_h_ch1:.0f}")
    c.eq("ch1 疊圖累積 20 點", ov_before, 20)

    # ── 切到 ch2：ch1 的圖表歷史必須留著，統計換成 ch2 的 ──
    w.set_focus_channel("ch2")
    c.eq("焦點切到 ch2", w.focus_channel, "ch2")
    c.eq("★ch1 疊圖沒被清掉", len(w.alt_overlays["ch1"]["kh"]._x), ov_before)
    c.eq("ch2 統計從零開始（各板一套，不互相污染）", w.max_height, 0.0)

    for i in range(10):
        w.update_ui_from_zmq("ch2", mk(alt=500.0 + i * 5))
    max_h_ch2 = w.max_height
    c.check("ch2 累積自己的統計", max_h_ch2 > 0, f"max_height={max_h_ch2:.0f}")

    # ── 切回 ch1：統計必須原封不動回來 ──
    w.set_focus_channel("ch1")
    c.check("★切回 ch1 統計完整還原",
            abs(w.max_height - max_h_ch1) < 1e-6,
            f"expected {max_h_ch1:.1f}, got {w.max_height:.1f}")
    c.check("兩頻道疊圖都還在",
            len(w.alt_overlays["ch1"]["kh"]._x) > 0
            and len(w.alt_overlays["ch2"]["kh"]._x) == 10)

    w.set_focus_channel("ch2")
    c.check("重複切換 ch2 統計仍在", abs(w.max_height - max_h_ch2) < 1e-6)
    w.set_focus_channel("ch1")

    # ── 火工品按鈕：確認態文字變長，寬度不得變 ──
    btns = [w.pyro_flow.itemAt(i).widget() for i in range(w.pyro_flow.count())]
    pyro = [b for b in btns if isinstance(b, QPushButton) and not b.isCheckable()
            and (b.text().startswith("傘") or b.text().startswith("囊"))]
    c.eq("找到 6 顆火工品按鈕", len(pyro), 6)
    if pyro:
        b = pyro[0]
        w0 = b.width()
        b.click()
        c.check("兩段式保險存在（第一擊只進確認態）", "確認" in b.text())
        c.eq("★按鈕寬度在文字變長後不變", b.width(), w0, f"{w0}px")
        b.click()   # <300ms 的第二擊 = 手抖，必須忽略
        c.check("防手抖：300ms 內的第二擊被忽略", "確認" in b.text())
        c.check("保險提示 tooltip 還在",
                bool(b.toolTip()) and "兩段式" in b.toolTip())

    # ── reset 才是真正的重來 ──
    w.reset_gui_state()
    c.eq("reset_gui_state 清空每頻道狀態", w.ch_view_state, {})
    c.eq("reset_gui_state 清空統計", w.max_height, 0.0)

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
