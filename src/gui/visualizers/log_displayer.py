import logging
import sys
from PyQt6.QtWidgets import QTextEdit
from PyQt6.QtGui import QTextCursor
from PyQt6.QtCore import QObject, pyqtSignal
from datetime import datetime

class LogSignalEmitter(QObject):
    log_received = pyqtSignal(str)

class LogDisplayer:
    def __init__(self, log_widget: QTextEdit):
        self.log_widget = log_widget
        self.log_widget.setReadOnly(True)
        # 💡 設定滿版高對比深黑色背景與極高可讀性 Consolas 字型
        self.log_widget.setStyleSheet(
            "QTextEdit { "
            "background-color: #0d0e12; "
            "color: #f0f0f0; "
            "font-family: 'Consolas', 'Courier New', monospace; "
            "font-size: 13px; "
            "line-height: 1.4; "
            "border: 1px solid #2a2d34; "
            "padding: 4px; "
            "}"
        )
        # ★2026-07-31：限制文件區塊數。
        #
        # _append_log 直接 QTextEdit.append()，而 QTextDocument 預設【無上限】——
        # 每一行都永久留在文件裡，記憶體與重繪成本一路長上去。
        #
        # 而這不是理論問題：communicator._process_data 對每一筆解析失敗的封包
        # 都會印一行 "Format error"，2026-07-20 飛測記錄到封包碎裂率 38%。
        #     2 Hz × 2 頻道 × 38% ≈ 每秒 1.5 行
        #     發射台待命 3 小時 ≈ 16000 行
        # 每行都是帶行內樣式的 HTML 區塊。QTextEdit 到那個量級就開始卡，
        # 捲動延遲、視窗重繪掉格 —— 偏偏那正是需要它反應快的時候。
        #
        # 5000 行在 1.5 行/秒下是約 55 分鐘的回溯，遠超過任何一次要往回翻的
        # 距離；而真正要查的完整紀錄在 raw log 與 CSV 裡，不靠這個視窗。
        self.log_widget.document().setMaximumBlockCount(5000)
        # ★2026-08-01：這個功能的取捨。
        #
        # jx06T 加的原版是【直接丟掉】符合關鍵字的訊息。抑制洗版是對的 ——
        # ch2 的 com14 已經重試 331 次、把其他訊息全埋掉。
        # 但「丟掉」讓埠真的死掉時完全無聲，而那正是最需要知道的時刻。
        #
        # 改成【折疊】：
        #   · 第一則永遠顯示 —— 你要知道它【什麼時候】開始出問題
        #   · 之後的收起來，但每 60 秒吐一行摘要（第幾次、已經多久）
        #   · 恢復連線時也講一聲
        # 這樣既不洗版，也不會安靜。「沒有新訊息」和「一直在重試」
        # 在畫面上永遠分得出來。
        self.hide_port_errors = False
        self._retry_n = 0            # 本輪已折疊幾則
        self._retry_t0 = None        # 本輪第一則的時間
        self._retry_last = 0.0       # 上次吐摘要的時間
        self.emitter = LogSignalEmitter()
        self.emitter.log_received.connect(self._append_log)
        self.setup_logging()

    _RETRY_SUMMARY_S = 60.0      # 折疊時每隔多久吐一行摘要

    def set_hide_port_errors(self, enabled: bool):
        """設定是否折疊串列埠連線失敗與重試日誌（不是丟掉，見 __init__）"""
        if not enabled and self._retry_n:
            self._append_log(f"🔊 序列埠重試 log 恢復顯示"
                             f"（折疊期間共 {self._retry_n} 則）")
        self.hide_port_errors = enabled
        self._retry_n = 0
        self._retry_t0 = None
        self._retry_last = 0.0

    def _is_port_retry_log(self, msg: str) -> bool:
        """判斷訊息是否為串列埠連線失敗或重試的日誌"""
        retry_keywords = [
            "無法連線到 port",
            "Retrying...",
            "is offline. Attempting to connect",
            "connection lost! Starting reconnection loop",
            "Serial connection is not active. Initiating connection"
        ]
        return any(kw in msg for kw in retry_keywords)
    
    def _format_html_log(self, msg: str) -> str:
        """將純文字 log 轉換為高對比度、高可讀性的富文本 HTML 格式"""
        import html
        escaped_msg = html.escape(msg)

        # 核心可讀性原則：主體內文維持高對比純白 (#f0f0f0)，時間戳為鋼灰 (#7e8a9b)
        time_color = "#7e8a9b"
        tag_color = "#b0bec5"
        body_color = "#f0f0f0"  # 確保主要文字高對比、清爽可讀

        # 僅針對前綴關鍵字標籤進行鮮明色彩提示 (極高對比粗體)
        if "ERROR" in escaped_msg or "FAIL" in escaped_msg or "timed out" in escaped_msg:
            tag_color = "#ff4d4d"   # 鮮豔強烈紅
        elif "WARNING" in escaped_msg or "WARN" in escaped_msg or "stale" in escaped_msg:
            tag_color = "#ffc107"   # 明亮金黃
        elif "SUCCESS" in escaped_msg or "OK" in escaped_msg or "resumed" in escaped_msg:
            tag_color = "#00e676"   # 高亮鮮綠
        elif "[CMD]" in escaped_msg or "Transmitting" in escaped_msg:
            tag_color = "#00b0ff"   # 天藍
        elif "[STAGE]" in escaped_msg or "STAGE" in escaped_msg:
            tag_color = "#d500f9"   # 霓虹紫
        elif "ROCKET MSG" in escaped_msg:
            tag_color = "#76ff03"   # 嫩綠

        # 假設標準格式為 "HH:MM:SS [LEVEL] Message"
        if len(escaped_msg) > 8 and escaped_msg[2] == ':' and escaped_msg[5] == ':':
            timestamp = escaped_msg[:8]
            rest = escaped_msg[8:]
            close_bracket_idx = rest.find("]")
            tag_end = close_bracket_idx + 1 if close_bracket_idx != -1 else 12
            tag_part = rest[:tag_end]
            body_part = rest[tag_end:]

            return (
                f'<span style="color: {time_color}; font-family: consolas, monospace;">{timestamp}</span>'
                f'<span style="color: {body_color}; font-family: consolas, monospace;">'
                f'<b style="color: {tag_color};">{tag_part}</b>'
                f'{body_part}</span>'
            )
        else:
            return f'<span style="color: {body_color}; font-family: consolas, monospace;">{escaped_msg}</span>'

    def _append_log(self, msg: str):
        if self.hide_port_errors and self._is_port_retry_log(msg):
            import time as _t
            now = _t.time()
            if self._retry_t0 is None:
                # 第一則一定要看得到 —— 這是「什麼時候開始壞的」
                self._retry_t0 = now
                self._retry_last = now
                self._retry_n = 1
            else:
                self._retry_n += 1
                if now - self._retry_last >= self._RETRY_SUMMARY_S:
                    self._retry_last = now
                    mins = (now - self._retry_t0) / 60.0
                    self._append_log(
                        f"🔇 序列埠仍在重試（已折疊 {self._retry_n} 則，"
                        f"持續 {mins:.1f} 分鐘）—— 取消勾選可展開")
                return          # 折疊，但上面那行摘要已經出去了
        elif self._retry_t0 is not None and not self._is_port_retry_log(msg):
            pass                # 有其他訊息不代表重試停了，計數繼續
        html_formatted = self._format_html_log(msg)
        self.log_widget.append(html_formatted)
        self.log_widget.moveCursor(QTextCursor.MoveOperation.End)

    def setup_logging(self):
        # 創建自定義處理器
        qt_handler = self.QtLogHandler(self.emitter)
        qt_handler.setFormatter(logging.Formatter(
            '%(asctime)s [%(levelname)s] %(message)s',
            datefmt='%H:%M:%S'
        ))
        
        # 添加到root logger
        logging.getLogger().addHandler(qt_handler)
        
        # 重定向標準輸出
        sys.stdout = self.QtOutputRedirector(self.emitter)
        sys.stderr = self.QtOutputRedirector(self.emitter)
        
    class QtLogHandler(logging.Handler):
        def __init__(self, emitter: LogSignalEmitter):
            super().__init__()
            self.emitter = emitter
            
        def emit(self, record):
            try:
                msg = self.format(record)
                self.emitter.log_received.emit(msg)
            except Exception:
                self.handleError(record)
    
    class QtOutputRedirector:
        def __init__(self, emitter: LogSignalEmitter):
            self.emitter = emitter
            
        def write(self, text):
            if text.strip():
                timestamp = datetime.now().strftime('%H:%M:%S')
                self.emitter.log_received.emit(f'[{timestamp}] {text.strip()}')
                
        def flush(self):
            pass