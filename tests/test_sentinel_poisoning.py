# -*- coding: utf-8 -*-
"""sentinel 毒化的回歸測試（缺陷已於 2026-07-30 修復）。

改壞 start() 就會紅。以下是它當初的樣子：

communicator.py 的 stop() 順序：
    第 199 行  self.running = False
    第 210 行  self.data_queue.put(None)       ← sentinel

_process_data() 的迴圈是 `while self.running:`，所以在 199 之後、210 之前
就可能跳出迴圈 —— 那顆 None 沒人消費，留在 queue 裡。而 start() 不重建
queue（它在 __init__ 就建好），下一次的 parser thread 第一次 get() 拿到那顆
舊 None → break → 當場死掉。

症狀是**完全靜默**：序列埠照讀、raw log 檔案照長，但 GUI 一筆遙測都收不到，
而且沒有任何錯誤訊息（break 是正常退出）。檢查 raw log 還會以為資料正常在收。
queue 從此無上限積壓。

觸發途徑（backend_daemon.py 三處都是 stop() 緊接 start()）：
    set_port   :111/113    GUI 改 COM port
    set_baud   :119/121    GUI 改 baudrate
    reconnect  :126/127    GUI 手動重連

修法（已套用）：start() 裡加一行

    self.data_queue = queue.Queue()

修在 start() 而不是去調 stop() 的順序，是因為根因不是「sentinel 有沒有被領走」，
而是「上一輪的殘留會不會被下一輪看到」—— 只要 consumer 因為任何理由提前退出，
殘留都會傳下去。重建佇列是無條件正確的。
"""
import sys, time
from _common import Checker, REPO

sys.path.insert(0, str(REPO))

FRAME = (b"T12345 SQ42 AX+0.012 AY-0.003 AZ+0.998 GX+0.10 GY-0.20 GZ+0.05 "
         b"P1013.25 RH0.5 KH0.3 VZ+0.01 GA1.00 ST:0 MOD:F GPS:0,0 C:0 "
         b"VF8.12 VA0.00\r\n")

TRIALS = 20


class FakeSerial:
    is_open = True
    def readline(self): time.sleep(0.02); return FRAME
    def close(self): self.is_open = False
    def write(self, d): return len(d)
    def flush(self): pass


def run():
    c = Checker("communicator sentinel 毒化（回歸測試）")
    from src.core.communicator import SerialCommunicator

    dead = 0
    for _ in range(TRIALS):
        got = []

        class Obs:
            def on_data_received(self, d): got.append(d)
            def on_error(self, e): pass
            def on_connection_status_changed(self, *a, **k): pass

        comm = SerialCommunicator("FAKE", 9600)
        comm.serial = FakeSerial()
        comm._reconnect = lambda: None          # 別去開真的 COM port
        comm.add_observer(Obs())

        comm.start()
        time.sleep(0.15)
        comm.serial = FakeSerial()
        comm.stop()                              # ← 這裡可能留下 sentinel
        comm.serial = FakeSerial()
        comm.start()                             # ← parser 可能立刻死掉

        before = len(got)
        time.sleep(0.25)
        if len(got) - before == 0:
            dead += 1

        comm.running = False
        try:
            comm.stop()
        except Exception:
            pass

    c.check(f"{TRIALS} 次 stop→start 之後 parser thread 都還活著",
            dead == 0,
            f"死掉 {dead}/{TRIALS} 次（{dead*100//TRIALS}%）"
            + ("　← 修法：start() 裡加 self.data_queue = queue.Queue()" if dead else ""))

    # 修好之後這條也該成立：start() 應該給一條乾淨的 queue
    import inspect
    src = inspect.getsource(SerialCommunicator.start)
    c.check("start() 會重建 data_queue", "data_queue" in src,
            "沒有的話，殘留的 sentinel 與前一個埠的舊資料都會被繼承")

    return c.done()


if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
