# -*- coding: utf-8 -*-
"""IGNITION 的推導（2026-08-01）。

原本的寫法是「ST:0 期間第一個 GA>1.5 就標 IGNITION」。那是錯的 ——
ST:0 是【整段發射台待命時間】，搬上發射架、放下、有人撞到、陣風，
任何一次超過 1.5g 都會標下去。而 mark_derived 是一次性的，
誤標之後真正的點火再也蓋不掉。

改成：ST:0 期間只記【候選】（最後一次由下往上穿過門檻的時刻，一直覆寫），
等火箭自己確認離架（ST:1）才定案。距離太遠的候選不採信 ——
寧可讓 IGNITION 維持「—」，也不要標一個編出來的時間。
"""
import sys
from datetime import datetime, timedelta
from _common import Checker, main_window, frame

sys.path.insert(0, str(__import__("_common").REPO))


def run():
    c = Checker("IGNITION 推導")
    w = main_window()
    from src.core.models import SensorData

    T0 = datetime(2026, 8, 1, 13, 0, 0)

    def feed(t_s, ga, st, ms=None):
        """t_s = 相對秒；ga = 合加速度；st = ST"""
        d = SensorData.from_new_format(
            frame(t=int(10000 + t_s * 1000), sq=int(t_s * 2) + 1,
                  ga=f"{ga:.2f}", st=st),
            T0 + timedelta(seconds=t_s))
        w.update_ui(d)
        return d

    def ign_row():
        from src.gui.visualizers.stage_display import FLIGHT_SEQUENCE
        i = next(k for k, (n, _, _) in enumerate(FLIGHT_SEQUENCE) if n == "IGNITION")
        return w.stage_display.derived_rows.get(i)

    def fresh():
        w.reset_gui_state()
        w.stage_display.reset()

    # ── ① 發射台上的擾動不可以標成 IGNITION ────────────────────────
    fresh()
    for t, ga in ((0.0, 1.00), (0.5, 1.00), (1.0, 2.30), (1.5, 1.00),  # 搬動尖峰
                  (2.0, 1.00), (2.5, 1.00), (3.0, 1.00)):
        feed(t, ga, 0)
    c.check("★發射台上的 2.3g 尖峰【沒有】標成 IGNITION", ign_row() is None,
            f"實際 {ign_row()} —— ST:0 是整段待命時間，搬動就會超過門檻")
    c.check("但有記下候選（等離架確認）", w._ign_cand is not None)

    # ── ② 離架之後才定案，而且用的是【最後一個】上升沿 ─────────────
    for t, ga in ((3.5, 1.00), (4.0, 3.20)):     # ← 真正的點火在 4.0
        feed(t, ga, 0)
    feed(4.5, 5.10, 0)
    feed(5.3, 5.00, 1)                            # 離架偵測（2.5g×200ms 之後）
    r = ign_row()
    c.check("★離架後 IGNITION 才定案", r is not None, "應該要標出來了")
    if r:
        got = (r[1] - T0).total_seconds()
        c.check(f"★用的是最後一個上升沿（t=4.0）而非搬動尖峰（t=1.0），實得 t={got:.1f}",
                abs(got - 4.0) < 0.01,
                "搬動那次必須被真正的點火覆寫掉")

    # ── ③ 候選太舊就不採信，維持「—」──────────────────────────────
    fresh()
    feed(0.0, 3.00, 0)          # 很久以前的尖峰
    for t in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0):
        feed(t, 1.00, 0)        # 之後一直安靜（真正的上升沿掉包了）
    feed(11.0, 5.00, 1)         # 突然就離架了
    c.check("★候選距離離架 11 秒 → 不採信，維持「—」", ign_row() is None,
            f"實際 {ign_row()} —— 寧可空著，也不要標編出來的時間")

    # ── ④ 完全沒有候選（GUI 中途啟動）也不可以亂標 ──────────────────
    fresh()
    feed(0.0, 5.00, 1)          # 第一筆就已經在飛
    c.check("★沒有候選就不標", ign_row() is None)

    # ── ⑤ 正常時序：合理的前置時間會被採信 ─────────────────────────
    fresh()
    feed(0.0, 1.00, 0)
    feed(0.5, 4.00, 0)          # 點火
    feed(1.0, 5.00, 0)
    feed(1.5, 5.00, 1)          # 離架（前置 1.0 秒，符合 ~1.3s 的預期）
    r = ign_row()
    c.check("★正常前置 1.0 秒 → 採信", r is not None)
    if r:
        c.check("  時間戳正確", abs((r[1] - T0).total_seconds() - 0.5) < 0.01,
                f"{(r[1] - T0).total_seconds():.2f}")

    fresh()
    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
