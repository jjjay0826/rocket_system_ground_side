# -*- coding: utf-8 -*-
"""測試共用基礎設施。

⚠ 為什麼要有 singleton MainWindow
   MainWindow 內含真的 QOpenGLWidget 姿態顯示器。offscreen 平台下**重複建立**
   會死鎖——不是變慢，是整個 process 停住不動。這個雷踩過一次：某支測試每個
   case 建一個視窗，跑到第 3 個就卡死，掛了 80 分鐘才被發現。
   所以：全程只建一個，測試之間自行清理狀態。run_all.py 也因此把所有 GUI
   測試跑在同一個 process 裡。
"""
import os, sys, types, pathlib

# ── 路徑：從本檔位置推出 repo 根目錄，不寫死絕對路徑 ──
REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

_app = None
_window = None


def qt_app():
    """建立（或取回）唯一的 QApplication"""
    global _app
    if _app is None:
        from PyQt6.QtCore import Qt, QCoreApplication
        QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)
        from PyQt6.QtWidgets import QApplication
        _app = QApplication([])
    return _app


def _stub_location_displayer():
    """地圖元件在 offscreen 下沒有意義，換成 no-op，避免拉起 web engine"""
    if "src.gui.visualizers.location_displayer" in sys.modules:
        return
    stub = types.ModuleType("src.gui.visualizers.location_displayer")

    class LocationDisplayer:
        def __init__(self, *a, **k): pass
        def reset(self): pass
        def update(self, *a, **k): pass
        def add_event_marker(self, *a, **k): pass

    stub.LocationDisplayer = LocationDisplayer
    sys.modules["src.gui.visualizers.location_displayer"] = stub


def main_window(channels=("ch1", "ch2")):
    """取回唯一的 MainWindow（第一次呼叫時建立）"""
    global _window
    if _window is None:
        qt_app()
        _stub_location_displayer()
        from src.gui.main_window import MainWindow
        _window = MainWindow(list(channels))
    return _window


# ── 韌體 repo：跨 repo 協定測試要用。找不到就跳過，不讓測試整組失敗 ──
def firmware_repo():
    """回傳 rocket-system 的路徑，找不到回傳 None"""
    env = os.environ.get("ROCKET_FIRMWARE_REPO")
    candidates = ([pathlib.Path(env)] if env else []) + [
        REPO.parent / "rocket-system",
        pathlib.Path.home() / "Desktop" / "rocket-system",
    ]
    for c in candidates:
        if (c / "firmware-rocket" / "Core" / "Src" / "main.c").exists():
            return c
    return None


# ── 遙測封包樣板：欄位順序與韌體 lora_pkt 一致 ──
FRAME = ("T{t} SQ{sq} AX+0.012 AY-0.003 AZ+0.998 GX+0.10 GY-0.20 GZ+0.05 "
         "P1013.25 RH{rh} KH{kh} VZ{vz} GA{ga} ST:{st} MOD:{mod} GPS:1,9 C:{c} "
         "VF{vf:.2f} VA{va:.2f} LAT+22.17485 LON+120.89272")


def frame(t=1000, sq=1, rh="0.5", kh="0.3", vz="+0.01", ga="1.00",
          st=0, mod="F", c="0", vf=8.12, va=7.98):
    return FRAME.format(t=t, sq=sq, rh=rh, kh=kh, vz=vz, ga=ga,
                        st=st, mod=mod, c=c, vf=vf, va=va)


RESULTS = pathlib.Path(__file__).resolve().parent / "_results"


class Checker:
    """收集斷言結果。

    ⚠ 為什麼要寫檔案而不是只 print
       這台機器上 Qt 的 OpenGL 元件會在 process teardown 時把行程靜默殺掉
       ——exit code 0、沒有 traceback、**stdout 還沒 flush 的部分全部消失**。
       曾經因此以為測試「跑不起來」，其實它跑完了，只是輸出被吞掉。
       所以每一行都同時寫進 _results/<name>.log（buffering=1，逐行落地），
       run_all.py 以那個檔案判定成敗，不看 exit code。

    刻意不用 assert：一支測試要跑完全部案例再一次回報，否則第一個失敗
    就看不到後面還壞了什麼。
    """

    def __init__(self, title, log_name=None):
        self.title = title
        self.fails = []
        self.n = 0
        RESULTS.mkdir(exist_ok=True)
        stem = log_name or pathlib.Path(sys.argv[0]).stem or "test"
        self._f = open(RESULTS / f"{stem}.log", "w", encoding="utf-8", buffering=1)
        self._w("=" * 72)
        self._w(title)
        self._w("=" * 72)

    def _w(self, line):
        self._f.write(line + "\n")
        print(line, flush=True)

    def check(self, name, ok, note=""):
        self.n += 1
        self._w(("  ✓ " if ok else "  ✗ ") + name)
        if note:
            self._w("        " + note)
        if not ok:
            self.fails.append(name)
        return ok

    def eq(self, name, got, want, note=""):
        return self.check(name, got == want,
                          note or (f"got={got!r} want={want!r}" if got != want else ""))

    def skip(self, reason):
        self._w(f"  ⊘ 跳過：{reason}")
        return True

    def done(self):
        self._w("")
        if self.fails:
            self._w(f"RESULT FAIL [{self.title}]  {len(self.fails)}/{self.n} 失敗: {self.fails}")
        else:
            self._w(f"RESULT PASS [{self.title}]  {self.n}/{self.n}")
        self._f.close()
        return not self.fails
