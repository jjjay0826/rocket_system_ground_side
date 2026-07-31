# -*- coding: utf-8 -*-
"""IMU 軸向重映射（2026-08-01 定案：航電板豎放，感測器 −X 朝上）。

★這支測試存在的理由是【手性】。

把兩支軸對調而不補負號，會把右手座標翻成左手座標。加速度看不出來
（重力還是指同一個方向），但陀螺儀的旋轉方向會整個反過來 —— 火箭
順時針自旋，畫面畫逆時針，而且沒有任何東西會報錯。事後看資料也很難
發現，因為每一筆數字看起來都很合理。

所以六組預設每一組都要驗行列式 = +1，而不是只驗「az 有沒有變成 1g」。

實測依據（2026-08-01 桌上把板子從平放立起來）：
    加速度圖出現一個乾淨的台階，AX 從 0 → −1.00 g 並停住，GA 全程 1.00 g。
    加速度計量的是重力【反作用力】、指向上，所以 AX=−1 ⇒ −X 軸朝上。
當時畫面上的姿態是 Pitch −117.2° / Roll −104.6°、當前偏角 26.4°
（火箭躺在桌上不動）—— 那就是 atan2(ay, az) 兩項都趨近 0 的奇異點症狀。
"""
import sys, math
import numpy as np
from _common import Checker, main_window

sys.path.insert(0, str(__import__("_common").REPO))


def preset_matrix(preset):
    """把 ("+ay","+az","+ax",…) 前三項轉成 3x3 矩陣：new = M @ old"""
    idx = {"x": 0, "y": 1, "z": 2}
    M = np.zeros((3, 3))
    for row, token in enumerate(preset[:3]):          # ax, ay, az
        M[row, idx[token[-1]]] = -1.0 if token[0] == "-" else 1.0
    return M


def run():
    c = Checker("IMU 軸向重映射（手性 + 豎放實測值）")
    w = main_window()

    # ── ① 六組預設全部必須是右手座標 ──────────────────────────────
    for up, preset in w.AXIS_PRESETS.items():
        det = round(float(np.linalg.det(preset_matrix(preset))), 9)
        c.check(f"★{up} 是右手座標（det=+1）", det == 1.0,
                f"det={det}；-1 表示手性翻轉，陀螺儀旋轉方向會全部相反")
        acc, gyr = preset[:3], preset[3:]
        same = all(a[0] == g[0] and a[-1] == g[-1] for a, g in zip(acc, gyr))
        c.check(f"  {up} 陀螺儀排列與加速度一致", same, f"{acc} vs {gyr}")

    # ── ② 套用之後，站直的火箭 az 必須讀到 +1g ────────────────────
    for up in w.AXIS_PRESETS:
        M = preset_matrix(w.AXIS_PRESETS[up])
        raw = np.zeros(3)
        raw[{"x": 0, "y": 1, "z": 2}[up[-1]]] = 1.0 if up[0] == "+" else -1.0
        mapped_az = float((M @ raw)[2])
        c.check(f"★{up} 套用後 az = +1g（脫離 atan2 奇異點）",
                abs(mapped_az - 1.0) < 1e-9, f"az={mapped_az:+.3f}")

    # ── ③ 定案值：預設必須是 -x，而且【不再自動偵測】────────────────
    c.eq("★預設軸向 = -x（豎放實測）", w.AXIS_UP_DEFAULT, "-x")
    c.eq("目前套用的就是預設", w.axis_up, w.AXIS_UP_DEFAULT)
    for gone in ("_autodetect_axis", "axis_locked", "_axis_votes"):
        c.check(f"★不再有自動偵測的殘留：{gone}", not hasattr(w, gone),
                "自動偵測已於 2026-08-01 移除 —— 發射前不要有會自己改的設定")

    # ── ④ 用實測的原始讀值跑一次真正的姿態算式 ────────────────────
    # 這是最重要的一條：直接驗「螢幕上會不會顯示站直」。
    class D:
        pass

    d = D()
    d.ax, d.ay, d.az = -1.00, 0.02, -0.01      # 實測：豎放時 AX≈−1g
    d.gx = d.gy = d.gz = 0.0

    w.axis_config = dict(zip(w._AXIS_KEYS, w.AXIS_PRESETS["-x"]))
    ax = w._get_mapped_axis(d, "ax")
    ay = w._get_mapped_axis(d, "ay")
    az = w._get_mapped_axis(d, "az")
    c.check("映射後 az ≈ +1g", abs(az - 1.0) < 0.02, f"az={az:+.3f}")

    # update_ui 裡的算式，逐字照抄
    est_pitch = math.atan2(ay, az) * 180.0 / math.pi
    est_roll = -math.atan2(-ax, math.hypot(ay, az)) * 180.0 / math.pi
    c.check("★豎放的火箭顯示 Pitch ≈ 0°", abs(est_pitch) < 3.0,
            f"Pitch={est_pitch:+.1f}°（改之前畫面上是 −117.2°）")
    c.check("★豎放的火箭顯示 Roll ≈ 0°", abs(est_roll) < 3.0,
            f"Roll={est_roll:+.1f}°（改之前畫面上是 −104.6°）")

    # 對照組：如果沿用舊的 +z 映射，同一筆讀值會落在奇異點上
    w.axis_config = dict(zip(w._AXIS_KEYS, w.AXIS_PRESETS["+z"]))
    ax0 = w._get_mapped_axis(d, "ax")
    ay0 = w._get_mapped_axis(d, "ay")
    az0 = w._get_mapped_axis(d, "az")
    bad_roll = -math.atan2(-ax0, math.hypot(ay0, az0)) * 180.0 / math.pi
    c.check("★對照：舊的 +z 映射會給出 ±90° 的假傾斜",
            abs(abs(bad_roll) - 90.0) < 5.0,
            f"Roll={bad_roll:+.1f}° —— 這就是修之前的症狀")

    # ── ⑤ _get_mapped_axis 真的照 config 取值與變號 ────────────────
    w.axis_config = dict(zip(w._AXIS_KEYS, w.AXIS_PRESETS["-x"]))
    d.ax, d.ay, d.az = 0.10, 0.20, 0.30
    d.gx, d.gy, d.gz = 1.0, 2.0, 3.0
    c.check("-x 映射：az ← -ax", abs(w._get_mapped_axis(d, "az") + 0.10) < 1e-9)
    c.check("-x 映射：ax ← +az", abs(w._get_mapped_axis(d, "ax") - 0.30) < 1e-9)
    c.check("-x 映射：gz ← -gx", abs(w._get_mapped_axis(d, "gz") + 1.0) < 1e-9)

    # ── ⑥ settings.json 覆寫 ──────────────────────────────────────
    from src.utils.settings import load_axis_up, _VALID_AXIS_UP
    c.eq("load_axis_up 認得六個合法值", len(_VALID_AXIS_UP), 6)
    c.check("沒設定時回傳 None（→ 用寫死的預設）",
            load_axis_up() in (None,) + _VALID_AXIS_UP)

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
