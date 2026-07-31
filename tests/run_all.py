# -*- coding: utf-8 -*-
"""跑完所有測試。

⚠ 為什麼每支測試都開獨立 subprocess
   ① MainWindow 內含真的 QOpenGLWidget，同一個 process 裡**重複建立會死鎖**
      （不是變慢，是整個 process 停住；曾經掛了 80 分鐘才被發現）。
   ② 這台機器上 Qt 在 teardown 時會把 process 靜默殺掉 —— exit code 0、
      沒有 traceback、stdout 未 flush 的部分全部消失。所以 exit code 不可信。
   → 一支一個 process，成敗改看 _results/<name>.log 最後那行 RESULT。

用法：
    python tests/run_all.py                 全部
    python tests/run_all.py crossrepo link   只跑名稱含這些字的
    python tests/run_all.py --skip-known     跳過已知缺陷（CI 用）
"""
import sys, os, pathlib, subprocess

HERE = pathlib.Path(__file__).resolve().parent
RESULTS = HERE / "_results"

# 名稱、檔案、是否為「已知缺陷」（預期失敗，不計入整體成敗）
SUITES = [
    ("crossrepo", "test_crossrepo_protocol.py", False),
    ("parse",     "test_parse_and_zmq.py",      False),
    ("stage",     "test_stage_names.py",        False),
    ("focus",     "test_gui_focus.py",          False),
    ("reject",    "test_reject_dispatch.py",    False),
    ("link",      "test_link_battery.py",       False),
    ("sentinel",  "test_sentinel_poisoning.py", False),  # 2026-07-30 已修，見檔案開頭
]


def run_one(fname, timeout=300):
    """跑一支測試，回傳 (ok, 最後的 RESULT 行)。不看 exit code。"""
    log = RESULTS / (pathlib.Path(fname).stem + ".log")
    if log.exists():
        log.unlink()
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen", PYTHONIOENCODING="utf-8")
    try:
        subprocess.run([sys.executable, "-u", str(HERE / fname)],
                       cwd=str(HERE), env=env, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, "逾時（可能是 OpenGL 死鎖）"
    if not log.exists():
        return False, "沒有產生日誌 —— 測試在寫出任何結果之前就死了"
    lines = log.read_text(encoding="utf-8").splitlines()
    result = next((l for l in reversed(lines) if l.startswith("RESULT ")), None)
    if result is None:
        return False, "日誌沒有 RESULT 行 —— 測試中途死掉"
    return result.startswith("RESULT PASS"), result[len("RESULT "):]


def main(argv):
    skip_known = "--skip-known" in argv
    picks = [a for a in argv if not a.startswith("-")]
    RESULTS.mkdir(exist_ok=True)

    normal, known = [], []
    for name, fname, is_known in SUITES:
        if picks and not any(p in name or p in fname for p in picks):
            continue
        if is_known and skip_known:
            print(f"⊘ 跳過已知缺陷：{name}")
            continue
        print(f"▶ {name} …", flush=True)
        ok, msg = run_one(fname)
        print(f"   {msg}\n", flush=True)
        (known if is_known else normal).append((name, ok, msg))

    print("=" * 72)
    print("總結")
    print("=" * 72)
    for name, ok, _ in normal:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    for name, ok, _ in known:
        print(f"  {'PASS' if ok else '已知缺陷'}  {name}"
              + ("　← 修好了，把 SUITES 裡的 True 改成 False" if ok else ""))

    bad = [n for n, ok, _ in normal if not ok]
    print(f"\n完整輸出：{RESULTS}")
    if bad:
        print(f"✗ {len(bad)} 組失敗：{bad}")
        return 1
    print(f"✓ {len(normal)} 組全部通過"
          + (f"（另有 {len(known)} 組已知缺陷）" if known else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
