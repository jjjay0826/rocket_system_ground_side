# -*- coding: utf-8 -*-
"""「隱藏 Port 重試」的取捨（2026-08-01）。

jx06T 加的原版是【直接丟掉】符合關鍵字的訊息。抑制洗版是對的 ——
ch2 的 com14 已經重試 331 次，把其他訊息全埋掉了。

但「丟掉」讓埠真的死掉時完全無聲，而那正是最需要知道的時刻。
飛行中掉線代表遙測正在消失，而遙測是唯一可靠的飛行資料（SD 已實測
reset 鎖卡救不回來）。

所以這支守的是兩件事：
  ① 折疊不等於靜音 —— 第一則要看得到，之後要有週期摘要
  ② 一離架就強制解除
"""
import sys, time
from _common import Checker, main_window, frame

sys.path.insert(0, str(__import__("_common").REPO))

RETRY = "無法連線到 port 'com14' (FileNotFoundError: 系統找不到指定的檔案。)"
OTHER = "🚀 [ROCKET MSG] [INFO] something else"


def run():
    c = Checker("隱藏 Port 重試的取捨")
    w = main_window()
    ld = w.log_display

    shown = []
    real_append = ld.log_widget.append
    ld.log_widget.append = lambda html: shown.append(html)

    try:
        # ── ① 關閉時：照常全部顯示 ──────────────────────────────
        ld.set_hide_port_errors(False)
        shown.clear()
        for _ in range(5):
            ld._append_log(RETRY)
        c.eq("未折疊時 5 則全部顯示", len(shown), 5)

        # ── ② 開啟時：第一則仍要看得到 ──────────────────────────
        ld.set_hide_port_errors(True)
        shown.clear()
        ld._append_log(RETRY)
        c.eq("★折疊開啟後【第一則仍然顯示】", len(shown), 1,
             "要知道它什麼時候開始壞的")

        # ── ③ 之後的收起來 ──────────────────────────────────────
        n0 = len(shown)
        for _ in range(50):
            ld._append_log(RETRY)
        c.eq("後續 50 則被折疊", len(shown), n0)
        c.eq("計數有累積", ld._retry_n, 51)

        # ── ④ ★但不能永遠靜音：時間到要吐摘要 ────────────────────
        ld._retry_last -= ld._RETRY_SUMMARY_S + 1     # 假裝過了 60 秒
        shown.clear()
        ld._append_log(RETRY)
        c.eq("★超過 60 秒吐一行摘要", len(shown), 1)
        c.check("摘要講了折疊幾則", "52" in shown[0] or "折疊" in shown[0], shown[0][:90])
        c.check("摘要講了持續多久", "分鐘" in shown[0], shown[0][:90])

        # ── ⑤ 非重試訊息不受影響 ────────────────────────────────
        shown.clear()
        ld._append_log(OTHER)
        c.eq("★其他訊息照常顯示（折疊只針對重試）", len(shown), 1)

        # ── ⑥ 取消折疊時要報告期間發生了什麼 ────────────────────
        shown.clear()
        ld.set_hide_port_errors(False)
        c.check("★取消折疊時報告期間共幾則",
                any("恢復顯示" in x for x in shown), f"{len(shown)} 則")
        c.eq("計數歸零", ld._retry_n, 0)

        # ── ⑦ ★一離架就強制解除 ─────────────────────────────────
        from src.core.models import SensorData
        w._hide_retry_released = False
        w.hide_port_err_cb.setChecked(True)
        c.check("前置：折疊是開的", w.hide_port_err_cb.isChecked())

        w._hide_retry_flight_guard(SensorData.from_new_format(frame(t=1, sq=1, st=0)))
        c.check("地面（ST:0）不解除 —— 發射台上折疊是對的",
                w.hide_port_err_cb.isChecked())

        w._hide_retry_flight_guard(SensorData.from_new_format(frame(t=2, sq=2, st=1)))
        c.check("★離架（ST:1）強制解除",
                not w.hide_port_err_cb.isChecked(),
                "飛行中掉線 = 遙測正在消失，必須看得到")

        # 只做一次：操作員之後仍可自己再勾
        w.hide_port_err_cb.setChecked(True)
        w._hide_retry_flight_guard(SensorData.from_new_format(frame(t=3, sq=3, st=3)))
        c.check("解除只做一次，之後不再強制",
                w.hide_port_err_cb.isChecked(),
                "操作員自己再勾起來就尊重他的決定")

        # ── ⑧ ★守衛必須掛在【焦點分發之前】───────────────────────
        # update_ui 只對焦點頻道跑。而操作員會去勾這個框，正是因為某個
        # 頻道在洗版 —— 如果焦點就停在那個掛掉的頻道上，它一幀都不會
        # 進來，解除永遠不會觸發。掛在 update_ui_from_zmq 才是對的。
        import inspect
        src_zmq = inspect.getsource(w.update_ui_from_zmq)
        src_ui = inspect.getsource(w.update_ui)
        c.check("★守衛在 update_ui_from_zmq（每個頻道都會經過）",
                "_hide_retry_flight_guard" in src_zmq,
                "放在 update_ui 的話，焦點停在死掉的頻道上就永遠不解除")
        c.check("守衛不在 update_ui（那裡只有焦點頻道）",
                "_hide_retry_flight_guard" not in src_ui)
        i_guard = src_zmq.index("_hide_retry_flight_guard")
        i_focus = src_zmq.index("self.focus_channel")
        c.check("★守衛在焦點分發【之前】", i_guard < i_focus,
                f"guard@{i_guard} focus@{i_focus}")

        # 非焦點頻道離架也要能解除
        w._hide_retry_released = False
        w.hide_port_err_cb.setChecked(True)
        other = [ch for ch in w.channel_ids if ch != w.focus_channel]
        if other:
            w.update_ui_from_zmq(other[0],
                                 SensorData.from_new_format(frame(t=4, sq=4, st=1)))
            c.check(f"★非焦點頻道（{other[0]}）離架也解除",
                    not w.hide_port_err_cb.isChecked(),
                    "兩塊板在同一枚火箭上，任一塊離架就是離架")
    finally:
        ld.log_widget.append = real_append
        ld.set_hide_port_errors(False)
        w.hide_port_err_cb.setChecked(False)
        w._hide_retry_released = False

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
