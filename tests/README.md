# tests

```bash
python tests/run_all.py                  # 全部
python tests/run_all.py crossrepo link   # 只跑名稱含這些字的
python tests/run_all.py --skip-known     # 跳過已知缺陷（CI 用）
```

不需要 pytest，不需要接硬體。GUI 測試在 `QT_QPA_PLATFORM=offscreen` 下跑真的
`MainWindow`，不是 mock。

## 測試清單

| 名稱 | 檔案 | 守住什麼 |
|---|---|---|
| **crossrepo** | `test_crossrepo_protocol.py` | **韌體改遙測格式時當場叫**。直接從 `firmware-rocket/Core/Src/main.c` 挖出 `lora_pkt` 的 printf 格式字串，套值餵給本端解析器；同時比對 `FlightState_t` 列舉與 `protocol.h` 的巨集 |
| **parse** | `test_parse_and_zmq.py` | 截斷封包整幀丟棄、GPS 無座標降級、ZMQ socket 共用鎖且可重入 |
| **focus** | `test_gui_focus.py` | 切焦點頻道不毀另一頻道的圖表與統計、火工品按鈕寬度、兩段式保險 |
| **reject** | `test_reject_dispatch.py` | REJECT 只清真正被拒的那道指令、「已開傘」算證據不算失敗、跨頻道隔離 |
| **link** | `test_link_battery.py` | 送達率（含重開機偵測、樣本不足不下結論）、電量分級告警真的會觸發 |
| 🔴 **sentinel** | `test_sentinel_poisoning.py` | **已知缺陷，目前預期失敗**。見檔案開頭 |

## 跨 repo 測試需要韌體 repo

`crossrepo` 會去找 `rocket-system`，順序是：

1. 環境變數 `ROCKET_FIRMWARE_REPO`
2. 本 repo 的同層目錄 `../rocket-system`
3. `~/Desktop/rocket-system`

找不到就跳過，不會讓整組紅掉。

## 兩個踩過的雷（改測試前先看）

**① 一個 process 只能建一個 `MainWindow`。**
它內含真的 `QOpenGLWidget`，offscreen 下重複建立會**死鎖**——不是變慢，是整個
process 停住。曾經有一支測試每個 case 建一個視窗，跑到第 3 個卡死，掛了 80 分鐘
才被發現。所以 `run_all.py` 一支測試開一個 subprocess。

**② exit code 不可信，要看日誌。**
這台機器上 Qt 會在 teardown 時把 process 靜默殺掉：exit code **0**、沒有
traceback、**stdout 還沒 flush 的部分全部消失**。曾經因此以為測試「跑不起來」，
其實它跑完了只是輸出被吞掉。

所以 `Checker` 每一行都同時寫進 `_results/<name>.log`（`buffering=1`，逐行落地），
`run_all.py` 讀那個檔案最後的 `RESULT PASS` / `RESULT FAIL` 判定，不看 exit code。

`_results/` 已加入 `.gitignore`。

## 加新測試

```python
from _common import Checker, main_window, frame, firmware_repo

def run():
    c = Checker("這支在測什麼")
    c.check("條件描述", 實際結果 == 預期, "失敗時的補充說明")
    c.eq("另一種寫法", got, want)
    return c.done()

if __name__ == "__main__":
    import sys; sys.exit(0 if run() else 1)
```

然後在 `run_all.py` 的 `SUITES` 加一行。`_common` 提供：

- `main_window()` — singleton MainWindow（同 process 內共用）
- `frame(...)` — 產生一筆合法遙測字串，欄位順序與韌體一致
- `firmware_repo()` — 韌體 repo 路徑，找不到回 `None`
- `Checker` — 逐行落地的斷言收集器（**刻意不用 `assert`**：一支測試要跑完
  全部案例再一次回報，否則第一個失敗就看不到後面還壞了什麼）
