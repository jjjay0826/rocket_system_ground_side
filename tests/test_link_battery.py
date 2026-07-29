# -*- coding: utf-8 -*-
"""鏈路送達率（用 SQ 序號算）與電量分級告警。

原缺陷：電量告警整段是死碼 —— 它寫在「保險絲燒斷／已武裝」的提早 return
之後，永遠跑不到。電池快沒電正是最需要提前知道的事。
"""
import sys, logging
from _common import Checker, main_window, frame, REPO

sys.path.insert(0, str(REPO))


def run():
    c = Checker("鏈路送達率 + 電量分級告警")
    from src.core.models import SensorData
    w = main_window()
    w.reset_gui_state()

    # ── SQ 欄位解析 ──
    d = SensorData.from_new_format(frame(t=1000, sq=42, vf=8.1))
    c.eq("SQ 解析正確", d.lora_seq, 42)
    c.check("插入 SQ 後其他欄位未受影響",
            abs(d.v_fuse - 8.1) < 1e-6 and abs(d.location[0] - 22.17485) < 1e-4)

    old = ("T1000 AX+0.01 AY-0.00 AZ+0.99 GX+0.1 GY-0.2 GZ+0.0 P1013.25 "
           "RH0.5 KH0.3 VZ+0.01 GA1.00 ST:0 MOD:F GPS:0,0 C:0")
    c.eq("舊韌體封包 SQ=0，不參與統計",
         SensorData.from_new_format(old).lora_seq, 0)

    def feed(ch, seqs, vf=8.1):
        for s in seqs:
            w.update_ui_from_zmq(ch, SensorData.from_new_format(
                frame(t=s * 500, sq=s, vf=vf)))

    # ── 送達率 ──
    feed("ch1", range(1, 21))
    rate, _ = w.ch_link["ch1"]
    c.check("無掉包 → 100%", abs(rate - 1.0) < 1e-6, f"{rate*100:.0f}%")

    feed("ch2", range(1, 41, 2))          # 只收奇數 = 掉一半
    rate2, _ = w.ch_link["ch2"]
    c.check("掉一半 → 約 50%", 0.45 < rate2 < 0.56, f"{rate2*100:.0f}%")

    w.ch_link.clear(); w.ch_seq.clear()
    feed("ch1", range(100, 103))          # 只有 3 包
    c.check("樣本 <8 不下結論（剛連上不亂報）", "ch1" not in w.ch_link)

    w.ch_link.clear(); w.ch_seq.clear()
    feed("ch1", range(500, 530))          # 高序號
    feed("ch1", range(1, 21))             # 火箭重開機，序號歸零
    rate3, _ = w.ch_link["ch1"]
    c.check("火箭重開機不得誤判成大量掉包", rate3 > 0.9, f"{rate3*100:.0f}%")

    for r, want in ((1.0, "#66DD66"), (0.8, "#FFDD44"), (0.5, "#FF6666")):
        w.ch_link["ch1"] = (r, 40)
        txt, col = w._link_text("ch1")
        c.eq(f"送達率 {r*100:.0f}% → 顏色分級", col, want, txt)

    # ── 電量分級 ──
    for vf, want in ((8.4, "ok"), (7.4, "ok"), (7.1, "ok"),
                     (6.9, "low"), (6.5, "crit"), (-1, "na")):
        c.eq(f"{vf}V 分級", w._batt_level(vf), want)

    # ── 告警必須真的會觸發（舊版是死碼）──
    w._prev_batt_level.clear()
    w._prev_pyro_flags.clear()
    hits = []

    class Grab(logging.Handler):
        def emit(self, r): hits.append(r.getMessage())

    h = Grab()
    logging.getLogger().addHandler(h)
    try:
        w._track_pyro_power("ch1", SensorData.from_new_format(frame(t=1, sq=1, vf=8.2)))
        w._track_pyro_power("ch1", SensorData.from_new_format(frame(t=2, sq=2, vf=6.9)))
        c.check("★跌破 7.0V 觸發「電量偏低」", any("電量偏低" in x for x in hits))
        w._track_pyro_power("ch1", SensorData.from_new_format(frame(t=3, sq=3, vf=6.4)))
        c.check("★跌破 6.6V 觸發「電量危險」", any("電量危險" in x for x in hits))
        n = len(hits)
        w._track_pyro_power("ch1", SensorData.from_new_format(frame(t=4, sq=4, vf=6.4)))
        c.check("同一等級不重複洗版", len(hits) == n)
    finally:
        logging.getLogger().removeHandler(h)

    w.reset_gui_state()
    c.check("reset 清空鏈路與電量狀態",
            not w.ch_seq and not w.ch_link and not w._prev_batt_level)

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
