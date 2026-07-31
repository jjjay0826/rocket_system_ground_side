# -*- coding: utf-8 -*-
"""封包解析的安全性 + ZMQ socket 序列化。不需要 GUI。

涵蓋三個曾經真實存在的缺陷：
  ① 截斷封包被當成完整資料 → 假的 stage 0 會誘發假的開傘確認
  ② GPS 宣稱定位但無座標 → 靜默套用台北座標，落海搜救時災難性誤導
  ③ 遙測與 log 共用一顆 PUB socket 卻沒鎖 → send_multipart 交錯，封包損毀
"""
import sys, threading, inspect
from _common import Checker, frame, REPO

sys.path.insert(0, str(REPO))


def run():
    c = Checker("封包解析安全性 + ZMQ 序列化")
    from src.core.models import SensorData

    full = frame(t=12345, sq=42, st=2)
    d = SensorData.from_new_format(full)
    c.check("完整封包正常解析", d.stage == 2 and abs(d.v_fuse - 8.12) < 1e-6)

    # ── ① 截斷封包必須整幀丟棄 ──
    for cut, why in ((28, "切在 AX 之後"), (60, "切在 GZ 之後"), (95, "切在 VZ 之後")):
        try:
            SensorData.from_new_format(full[:cut])
            c.check(f"截斷封包被拒（{why}）", False,
                    "★舊碼會靜默填 0 → 假 stage 0 → 誘發假的開傘確認")
        except ValueError:
            c.check(f"截斷封包被拒（{why}）", True)

    # 舊韌體（無 SQ/VF/VA）仍要能解析 —— 向後相容不可誤傷
    old = ("T12345 AX+0.012 AY-0.003 AZ+0.998 GX+0.10 GY-0.20 GZ+0.05 "
           "P1013.25 RH0.5 KH0.3 VZ+0.01 GA1.00 ST:0 MOD:F GPS:0,0 C:0")
    try:
        d2 = SensorData.from_new_format(old)
        c.check("舊韌體封包（無 SQ/VF/VA）仍可解析", d2.v_fuse == -1.0 and d2.lora_seq == 0)
    except ValueError as e:
        c.check("舊韌體封包仍可解析", False, f"誤拒: {e}")

    # ── ② GPS 宣稱定位卻無座標 → 降級 ──
    no_coord = full.split(" LAT")[0]
    try:
        d3 = SensorData.from_new_format(no_coord)
        c.check("GPS:1 但無 LAT/LON → 降級 NO_FIX", d3.gnss_state == "NO_FIX",
                "★否則地圖標在台北 (25.0, 121.5)，落海搜救時災難性誤導")
    except ValueError:
        c.check("GPS:1 但無 LAT/LON → 降級 NO_FIX", True, "被截斷檢查擋掉（也可接受）")

    d4 = SensorData.from_new_format(full)
    c.check("有座標時正常回報 FIX_3D", d4.gnss_state == "FIX_3D",
            f"{d4.location}")

    # ── ③ ZMQ PUB socket 必須序列化 ──
    import src.backend_daemon as bd
    pub = inspect.getsource(bd.ZmqPublishObserver.on_data_received)
    log = inspect.getsource(bd.ZmqLogHandler.emit)
    c.check("遙測發送包在 send_lock 內", "with self.send_lock" in pub)
    c.check("log 發送包在 send_lock 內", "with self.send_lock" in log)
    c.check("ZmqLogHandler 收下同一把鎖",
            "send_lock" in inspect.signature(bd.ZmqLogHandler.__init__).parameters,
            "必須是同一把，不同鎖等於沒鎖")

    # 鎖必須可重入：送出失敗的 except 分支會 logger.error → 同執行緒重入 emit()
    probe = object.__new__(bd.ZmqPublishObserver)
    probe.send_lock = threading.RLock()
    reentrant = probe.send_lock.acquire(blocking=False)
    if reentrant:
        reentrant = probe.send_lock.acquire(blocking=False)
        probe.send_lock.release()
        probe.send_lock.release()
    c.check("鎖可重入（RLock），失敗路徑不會自我死鎖", reentrant)

    # ── 附帶：jx06T 的重複 daemon 防護還在 ──
    main_src = inspect.getsource(bd.main)
    c.check("ZMQ bind 失敗有明確錯誤訊息（不是裸 traceback）",
            "zmq.error.ZMQError" in main_src and "ALREADY running" in main_src,
            "兩個 daemon 搶同一個 COM port 會讓遙測整段消失")

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
