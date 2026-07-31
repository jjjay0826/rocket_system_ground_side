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

        self.emitter = LogSignalEmitter()
        self.emitter.log_received.connect(self._append_log)
        self.setup_logging()
    
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