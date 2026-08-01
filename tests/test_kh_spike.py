# -*- coding: utf-8 -*-
"""KH 單包突波不可以灌高頂點（2026-08-01）。

實測回報：KH 偶爾會出現【單包比鄰近值高 10m 以上】的突波。
原本 max_height = max(max_height, kh) 無條件吃進去，一包就把頂點永久
灌高 —— 而那個值會進 APOGEE 標籤、進畫面，也是報告書上要交的頂點高度。

改成要連續兩包背書。真正的爬升每一包都在漲，不受影響。
"""
import sys, logging
from _common import Checker, main_window, frame

sys.path.insert(0, str(__import__("_common").REPO))


def run():
    c = Checker("KH 單包突波")
    w = main_window()
    from src.core.models import SensorData

    def feed(kh):
        w.update_ui(SensorData.from_new_format(frame(t=1, sq=1, kh=f"{kh:.1f}")))

    # ── ① 正常爬升：每一包都要算進去 ──────────────────────────────
    w.reset_gui_state()
    for h in (10, 25, 60, 120, 200, 310, 420, 500):
        feed(h)
    c.check("★正常爬升每一包都算（只落後一包）", abs(w.max_height - 420) < 0.1,
            f"max_height={w.max_height:.1f} —— 落後一包是設計，500 還沒進來")
    feed(500)
    c.check("  下一包就補上", abs(w.max_height - 500) < 0.1, f"{w.max_height:.1f}")

    # ── ② ★單包突波不可以灌高 ────────────────────────────────────
    w.reset_gui_state()
    for h in (100, 150, 200):
        feed(h)
    base = w.max_height
    hits = []

    class Grab(logging.Handler):
        def emit(self, r): hits.append(r.getMessage())
    h_ = Grab(); logging.getLogger().addHandler(h_)
    try:
        feed(215)          # 突波（比前後都高 13~15m）
        feed(202)          # 回到正常
        feed(205)          # 再一包，讓 202 也進得去
        # ★斷言的是「突波值本身沒進去」，不是「max 沒漲」——
        #   同期間 200/202 是真實資料，本來就該漲。
        c.check("★突波值 215 沒有進入 max_height",
                w.max_height < 210,
                f"max_height={w.max_height:.1f}（真實值約 202，突波是 215）")
        c.check("  但同期間的真實上升有算到",
                w.max_height >= 200, f"{w.max_height:.1f}")
        c.check("★突波有記進 log（不是靜默丟掉）",
                any("KH 單包突波" in x for x in hits),
                "丟掉但不說，事後查不到氣壓計出過問題")
    finally:
        logging.getLogger().removeHandler(h_)

    # ── ③ 連續兩包都高 → 那是真的爬升，要採信 ─────────────────────
    w.reset_gui_state()
    for h in (100, 150, 200):
        feed(h)
    feed(260); feed(265); feed(270)
    c.check("★連續上升全部採信（不是每兩包只取一包）",
            abs(w.max_height - 265) < 0.1,
            f"{w.max_height:.1f} —— 第一版的 min() 寫法這裡會是 260")

    # ── ④ 突波之後真正的頂點仍要記得到 ────────────────────────────
    w.reset_gui_state()
    for h in (100, 300, 150, 320, 340):     # 300 是突波，340 是真頂點
        feed(h)
    feed(345)
    c.check("★突波之後真正的頂點仍記得到", w.max_height >= 340,
            f"{w.max_height:.1f} —— 過濾不能把真值也擋掉")

    w.reset_gui_state()
    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
