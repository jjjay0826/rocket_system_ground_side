import csv
import logging
from src.core.models import SensorData
from src.storage.base import DataStorage

# 欄位順序固定：舊檔案接著寫時 DictWriter 必須用同一份順序，
# 否則同一個檔案裡會出現兩種欄位排列，事後解析全亂。
FIELDNAMES = [
    "timestamp", "rotationRoll", "rotationPitch", "direction", "stage", "location",
    "timestamp_ms", "ax", "ay", "az", "gx", "gy", "gz", "pressure",
    "rel_height", "kfh_height", "vz", "total_accel", "temp", "raw_adc",
    "flight_state", "module_state", "gnss_state", "sv_visible", "sv_used",
    "buffer_val", "count_val", "cond_a_raw", "cond_a_eff", "cond_b_raw",
    "cond_b_eff", "peak_height", "sd_writes", "lora_seq", "lora_success", "lora_total"
]


class CsvDataStorage(DataStorage):
    """CSV 數據存儲。

    ★2026-08-01 審查後改寫。原本三個問題：

    ① 每一筆都 open → write → close。
       和 raw log 當初那個缺陷同一類（見 communicator._read_serial 的說明）。
       這裡跑在【解析執行緒】而不是序列埠讀取執行緒，所以卡住不會直接
       掉位元組 —— 但 data_queue 沒有上限，它會一路積上去。
       改成保持開檔，每筆 flush（2Hz 下成本可忽略，而且當掉不會丟資料）。

    ② 沒有指定 encoding。
       Windows 上會用地區預設（正體中文是 cp950）。module_state / gnss_state
       是字串欄位，只要出現一個 cp950 編不出來的字元就 UnicodeEncodeError
       → 那一筆【整列消失】，只留一行 log。改成明確 utf-8。

    ③ 失敗只記一行 log，沒有計數也沒有升級。
       SD 卡已實測不可靠，遙測 CSV 是飛行資料的最終記錄。磁碟滿了、檔案被
       防毒鎖住、路徑不存在 —— 每一筆都會失敗，而畫面上只會有一堆長得
       一樣的紅字滾過去，很容易被當成雜訊。現在會在第 1、10、100 筆和
       之後每 500 筆升級告警，並在恢復時明講恢復了。
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self._fh = None
        self._path = None
        self._writer = None
        self._fail_streak = 0

    def _close(self):
        if self._fh is not None:
            try:
                self._fh.close()
            except Exception:
                pass
        self._fh, self._writer, self._path = None, None, None

    def _row(self, data: SensorData) -> dict:
        # GPS 沒定位時 models 會填 (25.0, 121.5)＝台北，並把 gnss_state 降成
        # NO_FIX。那個預設值寫進 CSV 會變成一條「飛到台北又飛回來」的軌跡 ——
        # models.py 自己的註解就寫了「落海搜救時是災難性的誤導」。
        # 這裡順著同一個判斷：沒有定位就留空白格，空格一看就知道是缺值。
        loc = ""
        if data.location and getattr(data, "gnss_state", "") != "NO_FIX":
            loc = f"{data.location[0]},{data.location[1]}"
        return {
            "timestamp": data.timestamp,
            "rotationRoll": data.rotationRoll,
            "rotationPitch": data.rotationPitch,
            "direction": data.direction,
            "stage": data.stage,
            "location": loc,
            "timestamp_ms": data.timestamp_ms,
            "ax": data.ax, "ay": data.ay, "az": data.az,
            "gx": data.gx, "gy": data.gy, "gz": data.gz,
            "pressure": data.pressure,
            "rel_height": data.rel_height,
            "kfh_height": data.kfh_height,
            "vz": data.vz,
            "total_accel": data.total_accel,
            "temp": data.temp,
            "raw_adc": data.raw_adc,
            "flight_state": data.flight_state,
            "module_state": data.module_state,
            "gnss_state": data.gnss_state,
            "sv_visible": data.sv_visible,
            "sv_used": data.sv_used,
            "buffer_val": data.buffer_val,
            "count_val": data.count_val,
            "cond_a_raw": data.cond_a_raw,
            "cond_a_eff": data.cond_a_eff,
            "cond_b_raw": data.cond_b_raw,
            "cond_b_eff": data.cond_b_eff,
            "peak_height": data.peak_height,
            "sd_writes": data.sd_writes,
            "lora_seq": data.lora_seq,
            "lora_success": data.lora_success,
            "lora_total": data.lora_total,
        }

    def save(self, data: SensorData, filename: str):
        try:
            # 換檔（/reset-data 會改 storage_obs.filename）→ 收掉舊的開新的
            if self._fh is None or self._path != filename:
                self._close()
                self._fh = open(filename, "a", newline="", encoding="utf-8")
                self._path = filename
                self._writer = csv.DictWriter(self._fh, fieldnames=FIELDNAMES)
                if self._fh.tell() == 0:
                    self._writer.writeheader()

            self._writer.writerow(self._row(data))
            self._fh.flush()   # 2Hz 下成本可忽略，換來「當掉不丟已收到的資料」

            if self._fail_streak:
                self.logger.warning(
                    f"✅ CSV 寫入已恢復（先前連續失敗 {self._fail_streak} 筆）：{filename}")
                self._fail_streak = 0

        except Exception as e:
            self._fail_streak += 1
            self._close()          # 壞掉的 handle 不要留著，下一筆重開
            n = self._fail_streak
            if n in (1, 10, 100) or n % 500 == 0:
                self.logger.error(
                    f"🔴 CSV 寫入失敗第 {n} 筆（{filename}）：{e}。"
                    + ("" if n < 10 else
                       " 遙測是目前唯一可靠的飛行紀錄（SD 已知不可靠）——"
                       " 請檢查磁碟空間、資料夾權限與防毒軟體。"))
