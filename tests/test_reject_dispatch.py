# -*- coding: utf-8 -*-
"""REJECT 只能清掉「真正被拒的那道指令」。

原缺陷：舊版只比對頻道，所以一句 RECAL 的拒收會把待確認的**開傘指令**一起
清掉，而且把紅色告警貼上「DPL 被拒收」的錯誤標籤 —— 操作員會以為開傘失敗，
在傘其實正常的情況下做出錯誤處置。
"""
import sys
from _common import Checker, main_window, REPO

sys.path.insert(0, str(REPO))


def run():
    c = Checker("REJECT 分派：只清被拒的那道指令")
    w = main_window()

    def case(name, msg, pending, expect, note=""):
        w.pending_confirms.clear()
        w.ch_pyro_confirmed.clear()
        for a in pending:
            w._register_confirm(["ch1"], a)
        w._handle_rocket_msg("ch1", f"🚀 [ROCKET MSG] [WARN] {msg}")
        remaining = sorted(k[1] for k in w.pending_confirms if k[0] == "ch1")
        c.check(name, remaining == sorted(expect),
                note or f"待確認 {pending} → 剩 {remaining}（預期 {sorted(expect)}）")

    # ── 非火工品指令的拒收不得動到火工品的待確認 ──
    case("RECAL 拒收不影響火工品",
         "REJECT recal only allowed in IDLE", ["dpl"], ["dpl"])
    case("SETCH 拒收不影響火工品",
         "REJECT setch only allowed in IDLE", ["dpl", "abg"], ["dpl", "abg"])
    case("壞頻道拒收不影響火工品",
         "REJECT bad channel - use #CMD:SETCH_72# (0-80)", ["dpl"], ["dpl"])

    # ── 韌體的 pyro 拒收會自報指令名，只清該一道 ──
    case("dpl 被拒 → 只清 dpl",
         "REJECT dpl - ascent guard (<10s after launch)", ["dpl", "abg"], ["abg"])
    case("abg 被拒 → 只清 abg",
         "REJECT abg - already landed", ["dpl", "abg"], ["dpl"])
    case("dpl 未武裝被拒 → 只清 dpl",
         "REJECT dpl - IDLE and not armed - ARM first (bench unlock)",
         ["dpl", "abg"], ["abg"])

    # ── 舊韌體的拒收不帶指令名 → 保守全清 ──
    case("舊韌體無指令名 → 保守全清",
         "REJECT already landed", ["dpl", "abg"], [])

    # ── 「已開傘」是好消息，要當成確認不是失敗 ──
    w.pending_confirms.clear()
    w.ch_pyro_confirmed.clear()
    w._register_confirm(["ch1"], "dpl")
    w._handle_rocket_msg(
        "ch1", "🚀 [ROCKET MSG] [WARN] REJECT dpl - already deployed (chute is already out)")
    c.check("★『已開傘』判定為確認而非失敗",
            ("ch1", "dpl") in w.ch_pyro_confirmed
            and ("ch1", "dpl") not in w.pending_confirms,
            "burst 第 2~4 發必然收到這句；SUCCESS 幀掉包時它是唯一線索")

    # ── 跨頻道隔離 ──
    w.pending_confirms.clear()
    w.ch_pyro_confirmed.clear()
    w._register_confirm(["ch1"], "dpl")
    w._register_confirm(["ch2"], "dpl")
    w._handle_rocket_msg("ch2", "🚀 [ROCKET MSG] [WARN] REJECT dpl - already landed")
    c.check("ch2 的拒收不影響 ch1 的待確認",
            ("ch1", "dpl") in w.pending_confirms
            and ("ch2", "dpl") not in w.pending_confirms)

    w.pending_confirms.clear()
    w.ch_pyro_confirmed.clear()
    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
