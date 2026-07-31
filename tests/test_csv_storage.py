# -*- coding: utf-8 -*-
"""遙測落盤（2026-08-01 審查）。

SD 卡已實測 reset 鎖卡救不回來，這個 CSV 是飛行資料的最終記錄。
所以這支測的不只是「有沒有寫進去」，而是三件容易靜默失敗的事：

  · 換檔之後還寫不寫得下去（/reset-data 會改 filename）
  · 非 ASCII 內容會不會讓整列消失（原本沒指定 encoding，Windows 走 cp950）
  · 寫不進去的時候看不看得見（原本只有一行 log，2Hz 洗版等於沒有）
"""
import sys, csv, tempfile, logging, pathlib
from datetime import datetime
from _common import Checker, REPO

sys.path.insert(0, str(REPO))


class Grab(logging.Handler):
    def __init__(self):
        super().__init__()
        self.msgs = []

    def emit(self, r):
        self.msgs.append(r.getMessage())


def run():
    c = Checker("遙測落盤 CSV")
    from src.storage.csv_storage import CsvDataStorage, FIELDNAMES
    from src.core.models import SensorData
    from _common import frame

    tmp = pathlib.Path(tempfile.mkdtemp(prefix="csvtest_"))
    st = CsvDataStorage()

    def read(p):
        with open(p, newline="", encoding="utf-8") as f:
            return list(csv.DictReader(f))

    # ── ① 基本寫入 + 標頭只寫一次 ──────────────────────────────────
    f1 = str(tmp / "a.csv")
    for i in range(5):
        st.save(SensorData.from_new_format(frame(t=i, sq=i)), f1)
    rows = read(f1)
    c.eq("寫進 5 筆", len(rows), 5)
    c.eq("欄位齊全", set(rows[0].keys()), set(FIELDNAMES))
    with open(f1, encoding="utf-8") as f:
        c.eq("標頭只出現一次", f.read().count("timestamp,rotationRoll"), 1)

    # ── ② 保持開檔：不是每筆都 open/close ───────────────────────────
    c.check("★寫完之後 handle 仍然開著（不是每筆 open/close）",
            st._fh is not None and not st._fh.closed,
            "每筆 open/close 在防毒掃描或磁碟忙碌時會把解析執行緒卡住")
    c.check("每筆都有 flush（當掉不丟已收到的資料）",
            len(read(f1)) == 5, "沒 flush 的話這時候還在緩衝區裡")

    # ── ③ 換檔（/reset-data）────────────────────────────────────────
    f2 = str(tmp / "b.csv")
    st.save(SensorData.from_new_format(frame(t=99, sq=99)), f2)
    c.eq("★換檔後新檔案有 1 筆", len(read(f2)), 1)
    c.eq("換檔後舊檔案沒有被動到", len(read(f1)), 5)
    c.eq("換檔後 path 已更新", st._path, f2)

    # ── ④ 無 GPS 定位時不可寫出台北座標 ────────────────────────────
    d_nofix = SensorData.from_new_format(
        frame(t=1, sq=1).split(" LAT")[0])          # 砍掉 LAT/LON = 無定位
    c.eq("無定位時 gnss_state = NO_FIX", d_nofix.gnss_state, "NO_FIX")
    f3 = str(tmp / "c.csv")
    st.save(d_nofix, f3)
    row = read(f3)[0]
    c.check("★無定位時 location 欄留空，不寫 (25.0,121.5)",
            row["location"] == "",
            f"實際 {row['location']!r} —— 那是台北，畫在地圖上會誤導搜救")

    # 有定位時照常寫
    f4 = str(tmp / "d.csv")
    st.save(SensorData.from_new_format(frame(t=1, sq=1)), f4)
    c.check("有定位時 location 照常寫入",
            read(f4)[0]["location"].startswith("22.17"),
            read(f4)[0]["location"])

    # ── ⑤ 非 ASCII 不得讓整列消失 ──────────────────────────────────
    d = SensorData.from_new_format(frame(t=1, sq=1))
    d.module_state = "測試中文™"          # cp950 編不出 ™
    f5 = str(tmp / "e.csv")
    st.save(d, f5)
    rows = read(f5)
    c.check("★非 ASCII 欄位不會讓整列消失（utf-8）",
            len(rows) == 1 and "測試中文" in rows[0]["module_state"],
            f"{len(rows)} 列；原本沒指定 encoding 時這裡會 UnicodeEncodeError")

    # ── ⑥ 寫入失敗要看得見，而且不能洗版 ───────────────────────────
    h = Grab()
    logging.getLogger().addHandler(h)
    try:
        bad = str(tmp / "no_such_dir" / "x.csv")     # 資料夾不存在 → 必失敗
        st2 = CsvDataStorage()
        for i in range(12):
            st2.save(SensorData.from_new_format(frame(t=i, sq=i)), bad)
        errs = [m for m in h.msgs if "CSV 寫入失敗" in m]
        c.eq("★連續失敗 12 筆只告警 2 次（第 1、第 10）", len(errs), 2,
             f"實際 {len(errs)} 次：2Hz 下每筆都吼等於沒有吼")
        c.check("第 10 筆的告警有講清楚後果",
                any("唯一可靠的飛行紀錄" in m for m in errs))
        c.eq("失敗計數正確", st2._fail_streak, 12)
        c.check("失敗後不留著壞掉的 handle", st2._fh is None)

        # 恢復要講
        h.msgs.clear()
        st2.save(SensorData.from_new_format(frame(t=1, sq=1)), str(tmp / "ok.csv"))
        c.check("★恢復時明講已恢復", any("已恢復" in m for m in h.msgs))
        c.eq("恢復後計數歸零", st2._fail_streak, 0)
    finally:
        logging.getLogger().removeHandler(h)

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
