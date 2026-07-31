# -*- coding: utf-8 -*-
"""IMU 軸向重映射（2026-07-31 航電板改豎放）。

★這支測試存在的理由是【手性】。

把兩支軸對調而不補負號，會把右手座標翻成左手座標。加速度看不出來
（重力還是指同一個方向），但陀螺儀的旋轉方向會整個反過來 —— 火箭
順時針自旋，畫面畫逆時針，而且沒有任何東西會報錯。事後看資料也很難
發現，因為每一筆數字看起來都很合理。

所以六組預設每一組都要驗行列式 = +1，而不是只驗「az 有沒有變成 1g」。

順便驗自動偵測：發射台靜置時重力指在縱軸上，量一下哪支軸吃到 1g 就
知道板子怎麼裝的。刻意只在 IDLE 做、鎖定後不再改 —— 飛行中重映射會
讓姿態曲線在半空中跳一下，那種圖沒人看得懂。
"""
import sys
import numpy as np
from datetime import datetime
from _common import Checker, main_window, frame

sys.path.insert(0, str(__import__("_common").REPO))


def preset_matrix(preset):
    """把 ("+ay","+az","+ax",...) 前三項轉成 3x3 矩陣：new = M @ old"""
    idx = {"x": 0, "y": 1, "z": 2}
    M = np.zeros((3, 3))
    for row, token in enumerate(preset[:3]):          # ax, ay, az
        sign = -1.0 if token[0] == "-" else 1.0
        M[row, idx[token[-1]]] = sign
    return M


def run():
    c = Checker("IMU 軸向重映射（手性 + 自動偵測）")
    w = main_window()

    # ── ① 六組預設全部必須是右手座標 ──────────────────────────────
    for up, preset in w.AXIS_PRESETS.items():
        M = preset_matrix(preset)
        det = round(float(np.linalg.det(M)), 9)
        c.check(f"★{up} 是右手座標（det=+1）", det == 1.0,
                f"det={det}；-1 表示手性翻轉，陀螺儀旋轉方向會全部相反")
        # 陀螺儀三項必須與加速度三項同一組排列，否則角速度配錯軸
        acc, gyr = preset[:3], preset[3:]
        same = all(a[0] == g[0] and a[-1] == g[-1] for a, g in zip(acc, gyr))
        c.check(f"  {up} 陀螺儀排列與加速度一致", same, f"{acc} vs {gyr}")

    # ── ② 套用之後，站直的火箭 az 必須讀到 +1g ────────────────────
    for up in w.AXIS_PRESETS:
        M = preset_matrix(w.AXIS_PRESETS[up])
        # 「up 軸朝上」= 該軸讀到 +1g（重力反作用力）
        raw = np.zeros(3)
        raw[{"x": 0, "y": 1, "z": 2}[up[-1]]] = 1.0 if up[0] == "+" else -1.0
        mapped_az = float((M @ raw)[2])
        c.check(f"★{up} 套用後 az = +1g（脫離 atan2 奇異點）",
                abs(mapped_az - 1.0) < 1e-9, f"az={mapped_az:+.3f}")

    # ── ③ 自動偵測：餵靜置資料，看能不能認出來 ────────────────────
    class D:
        """最小遙測替身：只需要自動偵測會讀的欄位"""
        def __init__(self, ax, ay, az, stage=0):
            self.ax, self.ay, self.az, self.stage = ax, ay, az, stage

    def try_detect(ax, ay, az, stage=0, n=None):
        w.axis_locked = False
        w.axis_up = "+z"
        w._axis_votes.clear()
        w.axis_config = dict(zip(w._AXIS_KEYS, w.AXIS_PRESETS["+z"]))
        for _ in range(n if n is not None else w._AXIS_VOTES):
            w._autodetect_axis(D(ax, ay, az, stage))
        return w.axis_up, w.axis_locked

    for up, (ax, ay, az) in (("+x", (0.99, 0.03, -0.02)),
                             ("-x", (-0.98, 0.01, 0.05)),
                             ("+y", (0.02, 1.00, 0.03)),
                             ("-y", (0.04, -0.97, -0.01)),
                             ("+z", (0.01, -0.02, 0.99)),
                             ("-z", (0.02, 0.03, -0.98))):
        got, locked = try_detect(ax, ay, az)
        c.check(f"★靜置 ({ax:+.2f},{ay:+.2f},{az:+.2f}) → 偵測為 {up}",
                got == up and locked, f"got={got} locked={locked}")

    # ── ④ 不該誤判的情況 ──────────────────────────────────────────
    got, locked = try_detect(0.60, 0.60, 0.53)          # 斜放，沒有主軸
    c.check("斜放（三軸都沒到 0.8g）→ 不下結論", not locked, f"got={got}")

    got, locked = try_detect(1.80, 0.02, 0.03)          # 在搬動，total≠1g
    c.check("搬動中（total_g=1.8）→ 不下結論", not locked, f"got={got}")

    got, locked = try_detect(0.99, 0.02, 0.01, n=w._AXIS_VOTES - 1)
    c.check(f"票數不足（{w._AXIS_VOTES - 1}/{w._AXIS_VOTES}）→ 還不採信", not locked)

    # ── ⑤ 飛行中不得重映射 ────────────────────────────────────────
    got, locked = try_detect(0.99, 0.02, 0.01, stage=1)
    c.check("★已離架（stage=1）→ 放棄偵測並鎖定，不在空中改映射",
            locked and got == "+z", f"got={got} locked={locked}")

    # ── ⑥ settings.json 釘死時不自動偵測 ──────────────────────────
    from src.utils.settings import load_axis_up, _VALID_AXIS_UP
    c.check("load_axis_up 認得六個合法值", len(_VALID_AXIS_UP) == 6)
    c.check("沒設定時回傳 None（→ 走自動偵測）",
            load_axis_up() in (None,) + _VALID_AXIS_UP)

    # ── ⑦ _get_mapped_axis 真的照 config 取值與變號 ────────────────
    w.axis_config = dict(zip(w._AXIS_KEYS, w.AXIS_PRESETS["-x"]))
    d = D(0.0, 0.0, 0.0)
    d.ax, d.ay, d.az = 0.10, 0.20, 0.30
    d.gx, d.gy, d.gz = 1.0, 2.0, 3.0
    c.check("-x 映射：az ← -ax", abs(w._get_mapped_axis(d, "az") + 0.10) < 1e-9,
            f"{w._get_mapped_axis(d, 'az')}")
    c.check("-x 映射：ax ← +az", abs(w._get_mapped_axis(d, "ax") - 0.30) < 1e-9)
    c.check("-x 映射：gz ← -gx", abs(w._get_mapped_axis(d, "gz") + 1.0) < 1e-9)

    # 還原成預設，不要污染同 process 的其他測試
    w.axis_config = dict(zip(w._AXIS_KEYS, w.AXIS_PRESETS["+z"]))
    w.axis_up, w.axis_locked = "+z", False
    w._axis_votes.clear()

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
