# -*- coding: utf-8 -*-
"""★ 最重要的一支：韌體改遙測格式時，讓本端當場知道自己脫節了。

做法不是比對「我以為的格式」，而是**直接從 firmware-rocket/Core/Src/main.c
把 lora_pkt 的 printf 格式字串挖出來**，套進實際數值，餵給本端的解析器。
韌體那邊改一個欄位，這支就會紅。

背景：shared/protocol.h、doc/telemetry_format.md、firmware-ground/README.md
三份文件都曾經描述一個 2026-06 就不存在的封包格式，沒有任何機制發現。
"""
import re, sys
from _common import Checker, firmware_repo, REPO

sys.path.insert(0, str(REPO))


def strip_c_comments(src):
    """把 C 註解換成等長空白。斷言「某識別字已經不存在」時一定要先過這道，
    否則留在註解裡的史料說明會讓測試永遠紅。"""
    out, i, n = [], 0, len(src)
    while i < n:
        if src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join(" " if ch != "\n" else "\n" for ch in src[i:j])); i = j
        elif src.startswith("//", i):
            j = src.find("\n", i); j = n if j < 0 else j
            out.append(" " * (j - i)); i = j
        else:
            out.append(src[i]); i += 1
    return "".join(out)


def run():
    c = Checker("跨 repo 協定：韌體格式 → 地面站解析")
    fw = firmware_repo()
    if fw is None:
        c.skip("找不到 rocket-system repo（可設環境變數 ROCKET_FIRMWARE_REPO 指定）")
        return c.done()

    src = (fw / "firmware-rocket" / "Core" / "Src" / "main.c").read_text(
        encoding="utf-8", errors="ignore")
    from src.core.models import SensorData

    STR = r'"((?:[^"\\]|\\.)*)"'
    fmts = [s for s in re.findall(STR, src) if s.startswith("T%lu SQ%lu AX")]
    c.eq("從 main.c 抓到兩種封包格式（有/無 GPS）", len(fmts), 2)
    if len(fmts) != 2:
        return c.done()

    SUBST = [("T%lu", "T12345"), ("SQ%lu", "SQ42"),
             ("AX%+0.3f", "AX+0.012"), ("AY%+0.3f", "AY-0.003"), ("AZ%+0.3f", "AZ+0.998"),
             ("GX%+0.2f", "GX+0.10"), ("GY%+0.2f", "GY-0.20"), ("GZ%+0.2f", "GZ+0.05"),
             ("P%.2f", "P1013.25"), ("RH%.1f", "RH12.5"), ("KH%.1f", "KH12.3"),
             ("VZ%+0.2f", "VZ-3.20"), ("GA%.2f", "GA1.05"),
             ("ST:%d", "ST:2"), ("MOD:%X", "MOD:F"), ("GPS:1,%u", "GPS:1,9"),
             ("C:%X", "C:F"), ("VF%.2f", "VF8.12"), ("VA%.2f", "VA7.98"),
             ("LAT%+0.5f", "LAT+22.17485"), ("LON%+0.5f", "LON+120.89272")]

    def build(fmt):
        s = fmt.replace("\\r\\n", "")
        for a, b in SUBST:
            s = s.replace(a, b)
        return s

    for fmt in fmts:
        has_gps = "LAT" in fmt
        label = "有 GPS 定位" if has_gps else "無 GPS 定位"
        line = build(fmt)
        # 格式字串裡不該還留著沒代換掉的 % —— 有的話代表韌體多了新欄位
        leftover = re.findall(r"%[-+0-9.]*[a-zA-Z]", line)
        c.check(f"{label}：所有欄位都認得", not leftover,
                f"未知欄位 {leftover}（韌體加了新欄位？）" if leftover else "")
        try:
            d = SensorData.from_new_format(line)
            ok = (d.stage == 2 and d.lora_seq == 42
                  and abs(d.kfh_height - 12.3) < 1e-6
                  and abs(d.vz + 3.20) < 1e-6
                  and abs(d.v_fuse - 8.12) < 1e-6
                  and abs(d.v_arm - 7.98) < 1e-6)
            c.check(f"{label}：解析後數值正確", ok,
                    f"ST={d.stage} SQ={d.lora_seq} KH={d.kfh_height} "
                    f"VZ={d.vz} VF={d.v_fuse} VA={d.v_arm} GPS={d.gnss_state}")
            if has_gps:
                c.check(f"{label}：座標正確", abs(d.location[0] - 22.17485) < 1e-4
                        and abs(d.location[1] - 120.89272) < 1e-4,
                        f"{d.location}")
        except Exception as e:
            c.check(f"{label}：解析", False, f"{type(e).__name__}: {e}")

    # ── ST 狀態碼必須與韌體的 FlightState_t 一致 ──
    m = re.search(r"typedef enum \{(.*?)\} FlightState_t;", src, re.S)
    if m:
        states = re.findall(r"(FLIGHT_[A-Z]+)", m.group(1))
        c.eq("韌體狀態機共 5 個狀態", len(states), 5,
             f"實際 {states}")
        c.eq("狀態順序", states,
             ["FLIGHT_IDLE", "FLIGHT_LAUNCHED", "FLIGHT_DEPLOYING",
              "FLIGHT_DEPLOYED", "FLIGHT_LANDED"],
             "doc/telemetry_format.md 的 ST 表必須跟著這個順序")

    # ── protocol.h 的 TX 巨集必須與 main.c 逐字相同 ──
    ph = fw / "shared" / "protocol.h"
    if ph.exists():
        h = ph.read_text(encoding="utf-8")

        def macro(name):
            i = h.index("#define " + name)
            out = []
            for ln in h[i:].split("\n"):
                out.append(ln)
                if not ln.rstrip().endswith("\\"):
                    break
            return "".join(re.findall(STR, "\n".join(out)))

        for name in ("RKT_LORA_TX_FMT_GPS", "RKT_LORA_TX_FMT_NOGPS"):
            try:
                c.check(f"protocol.h 的 {name} 與 main.c 逐字相同",
                        macro(name) in fmts)
            except ValueError:
                c.check(f"protocol.h 有 {name}", False, "巨集不存在")

    # ── 地面站據以自動確認火工品的 MSG 字串必須還在韌體裡 ──
    mw = (REPO / "src" / "gui" / "main_window.py").read_text(encoding="utf-8")
    for needle, why in (("Parachute deployed successfully", "確認遠端開傘"),
                        ("already deployed", "視為開傘證據")):
        c.check(f"「{needle}」兩端都有", needle in src and needle in mw, why)

    # ── ★2026-07-31 氣囊移除：反向守衛 ──────────────────────────────
    # 原本這裡驗的是「Airbag inflation started 兩端都有」。氣囊拆掉之後
    # 該驗的是相反的事：兩端都不可以再有任何會【發火】的氣囊路徑。
    c.check("韌體不再有氣囊充氣訊息", "Airbag inflation started" not in src,
            "訊息還在＝airbag_fire_auto 或 /abg 的發火路徑沒拆乾淨")
    c.check("地面站不再送出 abg", 'send_remote_cmd", ["abg"]' not in mw)
    for dead in ("airbag_fire_auto", "abg_active"):
        c.check(f"韌體無 {dead} 殘留", dead not in strip_c_comments(src),
                "PA0 現在是傘迴路的一半，任何單獨拉 PA0 的路徑都是活雷")
    # 反向：傘的兩支腳必須真的一起動
    c.check("★點傘一律走 deploy_fire_on()（PA0+PA1 同時）",
            "HAL_GPIO_WritePin(DEPLOY_PORT, DEPLOY_PIN, GPIO_PIN_SET)" not in src
            and src.count("deploy_fire_on()") >= 6,
            f"deploy_fire_on() 出現 {src.count('deploy_fire_on()')} 次（應 ≥6）")

    # 自動開傘的訊息**刻意不含** successfully，不該觸發下行確認
    auto = re.findall(r'"MSG SUCCESS Parachute deployed \([^"]*"', src)
    c.check("自動開傘訊息不含 'successfully'（不誤觸下行確認）",
            bool(auto) and all("successfully" not in a for a in auto),
            f"{len(auto)} 條自動開傘訊息")

    return c.done()


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
