import numpy as np
import logging
import threading
import math
import socket
import re
from datetime import datetime
from PyQt6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QCheckBox,
                             QLabel, QPushButton, QWidget)
from PyQt6.QtQuick import QQuickWindow, QSGRendererInterface
from PyQt6.QtCore import QTimer

from src.gui.ui_main import Ui_MainWindow  
from src.gui.qt_observer import QtGuiObserver
from src.gui.visualizers.line_chart import LineChartDrawer
from src.gui.flow_layout import FlowLayout
from src.gui.visualizers.stage_display import StageDisplayer
import logging
from src.gui.visualizers.log_displayer import LogDisplayer
from src.gui.visualizers.location_displayer import LocationDisplayer
from src.gui.visualizers.visualization_tools import euler_to_quaternion,quaternion_multiply
from src.gui.visualizers.attitude_displayer import AttitudeDisplayer, CubeGLWidget  
import zmq
import time
import json
from src.core.models import SensorData
from src.utils.settings import load_channel_settings, save_channel_settings

class MainWindow(QMainWindow):
    def __init__(self, channel_ids=None):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.angle_deviation = 0.0
        self.max_total_accel = 0.0
        self.max_deviation_angle = 0.0
        self.max_height = 0.0
        self.calib_q = None
        
        # ─── 快速軸向對應設定區 (Sensor Axis Mapping Configuration) ───
        # 定義：[火箭本體標準軸向] ➔ [感測器原始軸向] (支援正負號，例如 "-ay"、"+ax" 等)
        # 標準本體定義：Z_body=縱向自旋軸, X_body=橫向俯仰軸, Y_body=橫向偏航軸
        self.axis_config = {
            "ax": "+ax",  # 火箭 X 軸對應的感測器資料 (用以推算 Pitch)
            "ay": "+ay",  # 火箭 Y 軸對應的感測器資料 (用以推算 Roll)
            "az": "+az",  # 火箭 Z 軸對應的感測器資料 (用以對齊重力)
            "gx": "+gx",  # 橫向俯仰角速度 (Pitch Rate)
            "gy": "+gy",  # 橫向偏航角速度 (Yaw Rate)
            "gz": "+gz"   # 縱向滾轉/自旋角速度 (Roll/Spin Rate)
        }
        if channel_ids is None:
            channel_ids = ["ch1"]
        elif not isinstance(channel_ids, list):
            # 相容性包裝：若傳入的是 SerialCommunicator 或是其他型別，就用預設的 ch1
            channel_ids = ["ch1"]
        self.channel_ids = channel_ids

        # 載入通訊通道的序列埠與 ZMQ 埠設定
        self.channel_configs = {}
        for ch in self.channel_ids:
            port, baud, zmq_port, zmq_cmd_port = load_channel_settings(ch)
            self.channel_configs[ch] = {
                "port": port,
                "baud": baud,
                "zmq_port": zmq_port,
                "zmq_cmd_port": zmq_cmd_port
            }
        
        self.focus_channel = self.channel_ids[0]   # 勿寫死 "ch1":頻道名以設定為準
        self.start_time = time.time()
        self.last_recv_time = {ch: None for ch in self.channel_ids}
        self.channel_status = {ch: "No Data" for ch in self.channel_ids}

        # ── 下行閉環狀態(R1/R4)+格式錯誤偵測+VID/PID 快取 ──
        self.ch_last_fmt_err = {}     # ch -> 最近一次「解析失敗」時間戳(格式錯黃燈)
        self.ch_armed_until = {}      # ch -> ARMED 視窗截止時間(火箭 MSG 回讀)
        self.pending_confirms = {}    # (ch, action) -> 確認期限;逾期=LOUD 告警
        self.ch_pyro_confirmed = {}   # (ch, action) -> 火箭下行確認時間戳
        self._ports_cache = {}        # COM -> list_ports 資訊(VID/PID 識別)
        self._ports_scan_ts = 0.0
        self.ch_view_state = {}       # ch -> 該頻道的姿態/統計快照(切焦點時交換)
        self.ch_pyro_volt = {}        # ch -> (v_fuse, v_arm, 時間戳):pyro 電源監測
        self._prev_pyro_flags = {}    # ch -> (熔斷?, 已武裝?):只在狀態翻轉時發告警
        self.ch_prev_stage = {}       # ch -> 上一筆 stage(偵測「轉入」開傘的邊緣)

        self.latest_data = None

        self.last_valid_location = None
        self.last_valid_location_time = None
        self.est_pitch = 0.0
        self.est_roll = 0.0
        self.est_yaw = 180.0
        
        # 陀螺儀零點偏置與滑動視窗歷史 (靜止校準用)
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.gyro_history = []
        
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z

        # 💡 在主執行緒中直接建立 ZMQ SUB Socket 進行非阻塞輪詢，避免 Windows 下 QThread 與 Chromium Winsock 發生 Access Violation 記憶體衝突
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.SUB)
        
        connected_any = False
        for ch in self.channel_ids:
            try:
                _, _, zmq_port, _ = load_channel_settings(ch)
                address = f"tcp://127.0.0.1:{zmq_port}"
                self.zmq_socket.connect(address)
                self.zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")
                self.logger.info(f"Main thread connected to ZMQ PUB: {address}")
                connected_any = True
            except Exception as e:
                self.logger.error(f"Failed to connect to ZMQ PUB for channel {ch}: {e}")

        # 啟動 100Hz (10ms) 非阻塞資料輪詢定時器
        self.zmq_poll_timer = QTimer(self)
        self.zmq_poll_timer.timeout.connect(self.poll_zmq_data)
        self.zmq_poll_timer.start(10)

        # 啟動 5Hz 心跳偵測定時器
        self.heartbeat_timer = QTimer(self)
        self.heartbeat_timer.timeout.connect(self.check_heartbeats)
        self.heartbeat_timer.start(200) 
        
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        QQuickWindow.setGraphicsApi(QSGRendererInterface.GraphicsApi.OpenGL)

        # Chart 1：高度與速度 —— 純雙頻道對照圖(2026-07-22 使用者定案:
        # 焦點實線刪除,只畫 per-channel 覆疊曲線;所有頻道平權、全時段畫)。
        # curve_configs=[] → 無主曲線;update([]) 仍推 time_axis + auto_scroll
        # (X 軸捲動與 sync-X 依賴它)。
        self.chart_1 = LineChartDrawer(self.ui.chart_widget_1, window_width=200,
                                       curve_configs=[])
        # Chart 2：加速度
        self.chart_2 = LineChartDrawer(self.ui.chart_widget_2, window_width=200, curve_configs=[
            {'label': 'GA 總加速度(g)', 'color': (0, 180, 80),    'width': 3.0},
            {'label': 'AX (g)',         'color': (220, 60, 60),   'width': 1.5},
            {'label': 'AY (g)',         'color': (60, 120, 220),  'width': 1.5},
            {'label': 'AZ (g)',         'color': (230, 140, 0),   'width': 1.5},
        ])
        # Chart 3：姿態與角速度
        self.chart_3 = LineChartDrawer(self.ui.chart_widget_3, window_width=200, curve_configs=[
            {'label': 'Pitch 俯仰角(°)', 'color': (60, 120, 220),  'width': 2.0},
            {'label': 'Roll 滾轉角(°)',  'color': (220, 60, 60),   'width': 2.0},
            {'label': 'Yaw 旋轉角(°)',   'color': (60, 220, 60),   'width': 2.0},
            {'label': 'GX 角速度(°/s)', 'color': (0, 200, 200),   'width': 1.0},
            {'label': 'GY 角速度(°/s)', 'color': (200, 0, 200),   'width': 1.0},
            {'label': 'GZ 角速度(°/s)', 'color': (180, 180, 0),   'width': 1.0},
        ])

        # ── F3-A:非焦點頻道的高度/速度「同框」覆疊曲線(虛線)──
        # 每個頻道各建一組 KH+VZ 把手;只有「非焦點」頻道的資料會被推入
        # (update_ui_from_zmq),焦點頻道走主曲線。切焦點時全部清空重畫。
        _overlay_palette = [(150, 0, 200), (200, 120, 0), (0, 160, 160)]
        self.alt_overlays = {}
        for i, ch in enumerate(self.channel_ids):
            c = _overlay_palette[i % len(_overlay_palette)]
            self.alt_overlays[ch] = {
                "kh": self.chart_1.add_overlay_series(f"{ch} KH(m)", c, width=1.8),
                "vz": self.chart_1.add_overlay_series(f"{ch} VZ(m/s)",
                                                      (c[0]//2, c[1]//2, min(c[2]+80, 255)), width=1.2),
            }

        self.init_gui()

        self.stage_display = StageDisplayer(self.ui.listWidget)
        self.logger = logging.getLogger("src.gui.main_window")
        self.prev_health = {}  # 追蹤各模組健康狀態變化，僅在狀態轉換時記錄 log
        
        self.cube_widget = CubeGLWidget()
        self.ui.gl_gridLayout.addWidget(self.cube_widget)
        self.attitude_displayer = AttitudeDisplayer(self.cube_widget)

        self.ui.lineEdit.setPlaceholderText("Command-Line...")
        self.ui.lineEdit.returnPressed.connect(self.on_enter_pressed)
        
        self.location_displayer = LocationDisplayer(self.ui.map_widget)

        self.log_display = LogDisplayer(self.ui.log_textEdit) 

        # ── Per-channel 狀態 LED（左下,status_leds_layout 容器/.ui 定義)──
        # 每通道一組 [●LED][chN 文字]。顏色狀態機沿用 jx06 五態規則
        # (check_heartbeats),但改為逐通道渲染;收包瞬間亮綠脈衝由下一輪
        # heartbeats (200ms) 自然覆蓋回狀態色,不再需要 led_timer。
        self.ch_leds = {}
        for ch in self.channel_ids:
            led = QLabel()
            led.setFixedSize(12, 12)
            led.setStyleSheet("background-color: #555555; border-radius: 6px;")
            led.setToolTip(f"{ch}: no status yet")
            tag = QLabel(ch)
            tag.setStyleSheet("color: #999999; font-size: 10px;")
            self.ui.status_leds_layout.addWidget(led)
            self.ui.status_leds_layout.addWidget(tag)
            self.ch_leds[ch] = led

        # ── Per-channel port 狀態標籤(右下,multi_port_layout 容器)──
        # serial_label 保留顯示「焦點」頻道詳細字串;這排顯示所有頻道簡版。
        self.ch_port_labels = {}
        for ch in self.channel_ids:
            lbl = QLabel(f"{ch} --")
            lbl.setStyleSheet("color: #AAAAAA;")
            self.ui.multi_port_layout.addWidget(lbl)
            self.ch_port_labels[ch] = lbl

        # 非焦點頻道的最新高度快取(chart_label_1 併排顯示用,F3-B)
        self.ch_latest_alt = {}

        self.logger = logging.getLogger(__name__)

    def send_backend_command(self, cmd: str, args: list) -> bool:
        """對「當前焦點頻道」的後端發命令(單板)。"""
        return self.send_backend_command_to(self.focus_channel, cmd, args)

    def send_backend_command_to(self, target_ch: str, cmd: str, args: list) -> bool:
        """對「指定頻道」的後端 Daemon 發送控制命令 (ZMQ REQ + 超時防死鎖)。
        _all 廣播與單板命令共用此核心,只差 target_ch。"""
        cfg = self.channel_configs.get(target_ch)
        if not cfg:
            self.logger.error(f"No active config for channel {target_ch}")
            return False

        zmq_cmd_port = cfg.get("zmq_cmd_port")
        self.logger.info(f"Sending command '{cmd}' to backend daemon of {target_ch} on port {zmq_cmd_port}...")

        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, 5000) # 5000 毫秒接收超時 (相容 4 連發 0.7s 間隔)
        socket.setsockopt(zmq.SNDTIMEO, 5000) # 5000 毫秒傳送超時
        socket.connect(f"tcp://127.0.0.1:{zmq_cmd_port}")

        try:
            socket.send_json({"cmd": cmd, "args": args})
            reply = socket.recv_json()
            if reply.get("status") == "ok":
                self.logger.info(f"[{target_ch}] Command '{cmd}' executed successfully by backend.")
                return True
            else:
                error_msg = reply.get("error", "Unknown error")
                self.logger.error(f"[{target_ch}] Backend failed command '{cmd}': {error_msg}")
                return False
        except zmq.error.Again:
            self.logger.error(f"[{target_ch}] Backend command '{cmd}' timed out! Is the {target_ch} daemon running?")
            return False
        except Exception as e:
            self.logger.error(f"[{target_ch}] ZMQ command channel error: {e}")
            return False
        finally:
            try:
                socket.close()
                context.term()
            except Exception:
                pass

    def send_backend_command_all(self, cmd: str, args: list) -> bool:
        """對「所有航電板」(所有頻道)並行廣播命令——雙板熱備援同時觸發。
        並行(非序列)發送,兩板 pyro 幾乎同一時刻;逐板獨立,一板逾時/後端沒跑
        不影響另一板。

        ★安全語意(對抗性審查 R2/R10):成功 = 「所有」板都 TX-attempted 成功,
          不是「至少一板」。少一板 = 熱備援冗餘無聲消失,必須 LOUD 告警並點名
          失敗頻道。results 預先填 False:thread 拋例外/卡住時該板算失敗,不會
          被誤算成功。
        ★「TX-attempted」≠「已開傘」:回 ok 只代表 bytes 已寫入 COM,不代表火箭
          收到或 pyro 點燃——真正確認要看該板下行遙測 stage 是否轉開傘。"""
        chs = list(self.channel_ids)
        self.logger.warning(f"📡 [ALL] Broadcasting '{cmd}{args}' to {len(chs)} board(s): {chs}")
        results = {c: False for c in chs}   # 預填 False:未回報=失敗(安全方向)
        threads = []

        def _worker(c):
            try:
                results[c] = self.send_backend_command_to(c, cmd, args)
            except Exception as e:
                results[c] = False
                self.logger.error(f"[{c}] broadcast worker crashed: {e}")

        for ch in chs:
            t = threading.Thread(target=_worker, args=(ch,), daemon=True)
            t.start()
            threads.append(t)
        for t in threads:
            t.join(timeout=6)

        ok = sum(1 for v in results.values() if v)
        failed = [c for c, v in results.items() if not v]
        if ok == len(chs) and len(chs) > 0:
            self.logger.warning(f"📡 [ALL] TX-attempted to ALL {ok}/{len(chs)} boards. "
                                f"(‘sent’ ≠ ‘deployed’ — confirm by each board's downlink stage change.)")
            return True
        # ── 部分/全失敗:LOUD。冗餘可能已喪失,但單板仍安全(使用者確認:
        #    冗餘設計,單傘/單氣囊仍可安全著陸)→ 提示補點,不是中止。 ──
        self.logger.error(f"🔴🔴 [ALL] PARTIAL/FAILED broadcast: only {ok}/{len(chs)} boards accepted "
                          f"'{cmd}{args}'. FAILED: {failed}. HOT-STANDBY REDUNDANCY REDUCED (a single "
                          f"board still lands safely) — re-fire the failed board(s) individually to "
                          f"restore redundancy; do NOT abort.")
        return False

    # ══════════════════ 焦點切換與火工品按鈕列(F2) ══════════════════

    # 焦點頻道專屬狀態:切換時整包存起來、換上另一頻道的那包。
    # (姿態濾波/統計極值/校準基準本質上是「每塊板一套」,共用會互相污染)
    _PER_CH_STATE = ("latest_data", "last_valid_location", "last_valid_location_time",
                     "est_pitch", "est_roll", "est_yaw", "angle_deviation",
                     "max_total_accel", "max_deviation_angle", "max_height",
                     "calib_q", "quaternion", "gyro_bias_x", "gyro_bias_y",
                     "gyro_bias_z", "gyro_history", "prev_health")

    def _snapshot_focus_state(self) -> dict:
        return {k: getattr(self, k, None) for k in self._PER_CH_STATE}

    def _blank_focus_state(self) -> dict:
        """某頻道第一次成為焦點時的乾淨初值(欄位須與 _PER_CH_STATE 對齊)。"""
        return dict(latest_data=None, last_valid_location=None,
                    last_valid_location_time=None,
                    est_pitch=0.0, est_roll=0.0, est_yaw=180.0, angle_deviation=0.0,
                    max_total_accel=0.0, max_deviation_angle=0.0, max_height=0.0,
                    calib_q=None, quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                    gyro_bias_x=0.0, gyro_bias_y=0.0, gyro_bias_z=0.0,
                    gyro_history=[], prev_health={})

    def _apply_focus_state(self, snap: dict):
        for k, v in snap.items():
            setattr(self, k, v)

    def set_focus_channel(self, ch: str):
        """切換 GUI 渲染焦點頻道 = 換視角,不是重來。
        ★舊版切換直接 reset_gui_state(),圖表/統計/地圖全毀——來回切一次
          資料就沒了。現在改成:每頻道的姿態濾波與極值統計各自存一份,
          切換時整包交換;圖表歷史保留(切換點畫一條標記線標示分界)。"""
        if ch not in self.channel_ids:
            return
        # 按鈕視覺同步「先於」same-channel early return:checkable 按鈕被
        # 重複點擊時 Qt 已先 toggle 掉勾選,這裡撥回,否則畫面上沒有任何
        # 焦點鈕亮著、操作員看不出單板命令要打到哪塊板。
        for c, btn in getattr(self, "focus_buttons", {}).items():
            btn.setChecked(c == ch)
        if ch == self.focus_channel:
            return
        old = self.focus_channel
        self.ch_view_state[old] = self._snapshot_focus_state()      # 存舊的
        self.focus_channel = ch
        self._apply_focus_state(self.ch_view_state.get(ch) or self._blank_focus_state())
        self.logger.info(f"🔀 Focus channel switched: {old} -> {ch} "
                         f"(圖表歷史保留;{ch} 的統計已還原)")

        # 圖表上標一條分界線,免得操作員把兩頻道的曲線段誤讀成同一塊板
        x = time.time() - self.start_time
        for c in (self.chart_1, self.chart_2, self.chart_3):
            try:
                c.add_event_marker(x, f"▼{ch}", "#3D7BD9")
            except Exception:
                pass

        # 立刻用新頻道的狀態刷新姿態與極值顯示(否則要等下一筆遙測才更新)
        self.attitude_displayer.update(self.quaternion)
        self.ui.gl_label.setText(
            f"當前偏角: {self.angle_deviation:.1f}° | 最大偏角: {self.max_deviation_angle:.1f}°")

    def _build_pyro_button_row(self):
        """建構 log 與命令列之間的操作列(容器 pyro_button_row 由 .ui 提供):
        [焦點: ch1 ch2] ┃ [傘/囊 x 各板] ┃ [傘ALL 囊ALL] ┃ [校準] [Auto跟隨]
        點火鈕=兩段式防誤觸:第一按變紅倒數 3 秒,再按才發射,逾時還原。

        ★用 FlowLayout 而非 .ui 給的 QHBoxLayout:整排約 950px,寬螢幕一列排得
          下,但小筆電會溢出把右側圖表擠掉。FlowLayout 在寬度不足時自動折成
          兩列,不必為不同螢幕維護兩套版面。"""
        outer = self.ui.pyro_button_row      # .ui 提供的 QHBoxLayout
        holder = QWidget()
        row = FlowLayout(holder, margin=0, spacing=6)
        outer.addWidget(holder)
        self.pyro_flow = row                 # 測試/後續存取用

        lbl = QLabel("焦點:")
        lbl.setStyleSheet("color: #888888;")
        row.addWidget(lbl)
        self.focus_buttons = {}
        for ch in self.channel_ids:
            b = QPushButton(ch)
            b.setCheckable(True)
            b.setChecked(ch == self.focus_channel)
            b.setFixedHeight(26)
            b.setStyleSheet(
                "QPushButton{background:#333;color:#BBB;border:1px solid #555;border-radius:4px;padding:2px 10px;}"
                "QPushButton:checked{background:#1E5AA8;color:white;border-color:#3D7BD9;}")
            b.clicked.connect(lambda _, c=ch: self.set_focus_channel(c))
            row.addWidget(b)
            self.focus_buttons[ch] = b

        row.addWidget(self._vsep())
        # 單板點火鈕(每板:傘=dpl、囊=abg;走該板 backend 單發)
        for ch in self.channel_ids:
            row.addWidget(self._make_pyro_button(f"傘 {ch}", ch, "dpl"))
            row.addWidget(self._make_pyro_button(f"囊 {ch}", ch, "abg"))

        row.addWidget(self._vsep())
        # 廣播鈕(兩板同時;沿用 _all 的並行+LOUD 部分失敗告警)
        row.addWidget(self._make_pyro_button("傘 ALL", None, "dpl"))
        row.addWidget(self._make_pyro_button("囊 ALL", None, "abg"))

        row.addWidget(self._vsep())
        # 校準鈕:火箭端氣壓零點重校(全板)+地面端姿態歸零一鍵完成。
        # 韌體 IDLE 閘門擋掉飛行中誤按,故單擊即發、不套兩段式紅色確認。
        cal_btn = QPushButton("校準 ALL")
        cal_btn.setFixedHeight(26)
        cal_btn.setStyleSheet(
            "QPushButton{background:#1B3A4B;color:#7FD4E8;border:1px solid #2E5F73;"
            "border-radius:4px;padding:2px 10px;}"
            "QPushButton:hover{background:#25506A;}")
        cal_btn.clicked.connect(lambda: self._send_recal(broadcast=True))
        row.addWidget(cal_btn)

        # (FlowLayout 靠左排並自動折行,不需要 addStretch 撐開)

        # ── F5:全域「Auto 跟隨」——代理 4 顆原生 Auto(3 chart + map)──
        # 原 checkbox 隱藏但保留(update_ui 照舊讀它們,零改繪圖邏輯);
        # sync-X 仍在 chart1 標題列。
        self.global_auto_cb = QCheckBox("Auto 跟隨")
        self.global_auto_cb.setChecked(True)
        def _apply_auto(state):
            checked = (state == 2)
            for cb in (self.ui.chart_checkBox_1, self.ui.chart_checkBox_2,
                       self.ui.chart_checkBox_3, self.ui.map_checkBox):
                cb.setChecked(checked)
        self.global_auto_cb.stateChanged.connect(_apply_auto)
        row.addWidget(self.global_auto_cb)

    @staticmethod
    def _vsep():
        sep = QLabel("┃")
        sep.setStyleSheet("color: #444444;")
        return sep

    def _make_pyro_button(self, label: str, ch, action: str):
        """兩段式防誤觸點火鈕。ch=None 表示 ALL 廣播。
        第一按:紅色進入確認態+3 秒倒數(逾時自動還原);
        第二按(3 秒內):執行發射並還原。緊急時兩按 <1 秒即可送出,
        比彈窗快且不搶鍵盤焦點。"""
        btn = QPushButton(label)
        btn.setFixedHeight(26)
        idle_style = ("QPushButton{background:#402020;color:#D08080;border:1px solid #663333;"
                      "border-radius:4px;padding:2px 10px;}")
        armed_style = ("QPushButton{background:#CC2222;color:white;border:2px solid #FF5555;"
                       "border-radius:4px;padding:2px 10px;font-weight:bold;}")
        btn.setStyleSheet(idle_style)
        # ★寬度預先鎖死,兩種文字取較寬者:否則第一按文字變長→按鈕撐大→整排
        #   左右位移,第二按時手指落點已不是同一顆鈕(誤點鄰鈕)。
        #   確認態刻意用極短的「確認?」——早期版本寫「確認 傘 ch1?」,鎖出來的
        #   寬度是原本兩倍,六顆鈕就把整條操作列撐爆、擠掉右側圖表。
        #   按鈕位置不動 + 轉紅 + 粗體,已足夠辨識是哪一顆在等確認。
        _fm = btn.fontMetrics()
        btn.setFixedWidth(max(_fm.horizontalAdvance(label),
                              _fm.horizontalAdvance("確認?")) + 20)
        btn.setToolTip(
            f"{label} —— 兩段式防誤觸:\n"
            f"① 第一次按 → 變紅並顯示「確認 {label}?」,開始 3 秒倒數\n"
            f"② 3 秒內再按一次 → 才真正發射\n"
            f"• 300ms 內的連按視為手抖/雙擊,直接忽略\n"
            f"• 3 秒沒有第二次按 → 自動解除,回到安全狀態")
        state = {"armed": False, "armed_at": 0.0}
        timer = QTimer(self)
        timer.setSingleShot(True)

        def _disarm():
            state["armed"] = False
            btn.setText(label)
            btn.setStyleSheet(idle_style)

        def _fire():
            target = "ALL" if ch is None else ch
            self.logger.warning(f"🚨 [PYRO BUTTON] {action.upper()} -> {target}")
            self._register_confirm(self.channel_ids if ch is None else [ch], action)
            if ch is None:
                threading.Thread(
                    target=lambda: self.send_backend_command_all("send_remote_cmd", [action]),
                    daemon=True).start()
            else:
                threading.Thread(
                    target=lambda: self.send_backend_command_to(ch, "send_remote_cmd", [action]),
                    daemon=True).start()

        def _on_click():
            if state["armed"]:
                # ★300ms 最小武裝時間:Qt 雙擊=兩次完整 click,沒有下限的話
                #   手抖/觸控板雙擊會在 ~100ms 內武裝+發射,兩段式防護形同虛設。
                #   300ms 內的第二擊忽略(維持武裝態),刻意的兩連擊仍 <1s 完成。
                if time.monotonic() - state["armed_at"] < 0.3:
                    return
                timer.stop()
                _fire()
                _disarm()
            else:
                state["armed"] = True
                state["armed_at"] = time.monotonic()
                btn.setText("確認?")
                btn.setStyleSheet(armed_style)
                timer.start(3000)   # 3 秒未確認自動還原

        timer.timeout.connect(_disarm)
        btn.clicked.connect(_on_click)
        return btn

    def _send_recal(self, broadcast: bool = True):
        """發火箭端氣壓零點重校(#CMD:RECAL)並連帶做地面端姿態歸零——
        火箭與地面的零點一鍵對齊。韌體只在 IDLE 受理,飛行中誤發會被拒收
        (回 MSG WARN),無安全風險,故不走兩段式確認。"""
        scope = "ALL boards" if broadcast else "focus board"
        self.logger.info(f"🧭 [CAL] Transmitting baro re-zero (RECAL) to {scope}...")
        self.broadcast_event("[CMD] CAL", "#00E5FF")
        if broadcast:
            threading.Thread(
                target=lambda: self.send_backend_command_all("send_remote_cmd", ["cal"]),
                daemon=True).start()
        else:
            threading.Thread(
                target=lambda: self.send_backend_command("send_remote_cmd", ["cal"]),
                daemon=True).start()
        self._reset_angle_ground()

    def _reset_angle_ground(self):
        """地面端姿態/零偏歸零(原 /reset-angle 內聯邏輯抽出,供指令與 CAL 鈕共用)。"""
        if not self.latest_data:
            self.logger.error('No data received yet, cannot reset angle')
            return
        self.angle_deviation = self.latest_data.direction

        # 1. 依據映射後的加速度讀值推算出當前對地角度作為濾波器初始值 (自動校準)
        ax = self._get_mapped_axis(self.latest_data, "ax")
        ay = self._get_mapped_axis(self.latest_data, "ay")
        az = self._get_mapped_axis(self.latest_data, "az")
        try:
            roll_rad = math.atan2(ay, az)
            pitch_rad = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            self.est_pitch = roll_rad * 180.0 / math.pi
            self.est_roll = -pitch_rad * 180.0 / math.pi
        except Exception:
            self.est_pitch = 0.0
            self.est_roll = 0.0

        # 垂直於地面的旋轉角度 (Yaw) 則重置回到正前方 (180.0)
        self.est_yaw = 180.0

        # 2. 計算映射後的角速度均值作為靜態陀螺儀零點偏置 (Gyro Bias Calibration)
        if self.gyro_history:
            mapped_gyros = []
            for h_data in self.gyro_history:
                mgx = self._get_mapped_axis(h_data, "gx")
                mgy = self._get_mapped_axis(h_data, "gy")
                mgz = self._get_mapped_axis(h_data, "gz")
                mapped_gyros.append((mgx, mgy, mgz))

            self.gyro_bias_x = sum(g[0] for g in mapped_gyros) / len(mapped_gyros)
            self.gyro_bias_y = sum(g[1] for g in mapped_gyros) / len(mapped_gyros)
            self.gyro_bias_z = sum(g[2] for g in mapped_gyros) / len(mapped_gyros)
        else:
            self.gyro_bias_x = self._get_mapped_axis(self.latest_data, "gx")
            self.gyro_bias_y = self._get_mapped_axis(self.latest_data, "gy")
            self.gyro_bias_z = self._get_mapped_axis(self.latest_data, "gz")

        self.calib_q = self.handle_angle_change(self.est_pitch, self.est_yaw, self.est_roll)
        self.max_deviation_angle = 0.0
        self.max_total_accel = 0.0
        self.max_height = 0.0
        self.ui.gl_label.setText(
            "當前偏角: 0.0° | 最大偏角: 0.0°"
        )
        self.broadcast_event("[CMD] Reset Angle", "#00E5FF")
        self.logger.info(
            f"Angles calibrated: Yaw reset to 180.0, Pitch gravity={self.est_pitch:.2f}, Roll gravity={self.est_roll:.2f}. "
            f"Gyro Bias calibrated - X:{self.gyro_bias_x:.4f}, Y:{self.gyro_bias_y:.4f}, Z:{self.gyro_bias_z:.4f}"
        )

    def on_enter_pressed(self):
        text = self.ui.lineEdit.text().strip()
        if not text:
            return
            
        self.ui.lineEdit.clear()
        
        if text.startswith("/"):
            parts = text.split()
            cmd = parts[0].lower()
            args = parts[1:]
            
            if cmd == "/port":
                if not args:
                    self.logger.error("Usage: /port <PORT> (e.g. /port COM4)")
                    return
                new_port = args[0]
                self.logger.info(f"Requesting backend to switch port to {new_port}...")
                
                def run_port_switch():
                    success = self.send_backend_command("set_port", [new_port])
                    if success:
                        self.channel_configs[self.focus_channel]["port"] = new_port
                        self.logger.info(f"GUI updated focus port to {new_port}")
                
                threading.Thread(target=run_port_switch, daemon=True).start()
            elif cmd == "/baud":
                if not args:
                    self.logger.error("Usage: /baud <BAUDRATE> (e.g. /baud 115200)")
                    return
                try:
                    new_baud = int(args[0])
                    self.logger.info(f"Requesting backend to switch baudrate to {new_baud}...")
                    
                    def run_baud_switch():
                        success = self.send_backend_command("set_baud", [new_baud])
                        if success:
                            self.channel_configs[self.focus_channel]["baud"] = new_baud
                            self.logger.info(f"GUI updated focus baudrate to {new_baud}")
                    
                    threading.Thread(target=run_baud_switch, daemon=True).start()
                except ValueError:
                    self.logger.error("Invalid baudrate value. Must be an integer.")
            elif cmd == "/connect":
                self.logger.info("Requesting backend to reconnect serial...")
                threading.Thread(
                    target=lambda: self.send_backend_command("reconnect", []),
                    daemon=True
                ).start()
            elif cmd == "/disconnect":
                self.logger.info("Requesting backend to disconnect serial...")
                threading.Thread(
                    target=lambda: self.send_backend_command("disconnect", []),
                    daemon=True
                ).start()
            elif cmd == "/reset-angle":
                self._reset_angle_ground()
            elif cmd in ["/reset-data", "/reset"]:
                self.logger.info("Requesting backend to archive session data and create new log files...")

                def run_reset():
                    success = self.send_backend_command("reset_session", [])
                    if success:
                        self.logger.info("Backend data session reset successfully. Resetting UI state...")
                    else:
                        self.logger.warning("Backend reset session request failed or backend offline; resetting local UI state...")
                    
                    QTimer.singleShot(0, self.reset_gui_state)

                threading.Thread(target=run_reset, daemon=True).start()
            elif cmd == "/arm":
                self.logger.warning("🚨 [SAFETY] Transmitting remote SYSTEM ARM command (30s Unlock Window)...")
                self.broadcast_event("[CMD] ARM", "#FF9100")
                threading.Thread(
                    target=lambda: self.send_backend_command("send_remote_cmd", ["arm"]),
                    daemon=True
                ).start()
            elif cmd == "/dpl":
                self.logger.warning("🚨 [EMERGENCY] Transmitting remote FORCE PARACHUTE DEPLOYMENT command...")
                self.broadcast_event("[CMD] DPL", "#D500F9")
                self._register_confirm([self.focus_channel], "dpl")
                threading.Thread(
                    target=lambda: self.send_backend_command("send_remote_cmd", ["dpl"]),
                    daemon=True
                ).start()
            elif cmd == "/abg":
                self.logger.warning("🚨 [EMERGENCY] Transmitting remote AIRBAG DEPLOYMENT command...")
                self.broadcast_event("[CMD] ABG", "#1DE9B6")
                self._register_confirm([self.focus_channel], "abg")
                threading.Thread(
                    target=lambda: self.send_backend_command("send_remote_cmd", ["abg"]),
                    daemon=True
                ).start()
            # ── 雙板廣播:同時對所有航電板(ch1+ch2 熱備援)發命令 ──
            elif cmd == "/arm_all":
                self.logger.warning("🚨 [SAFETY] Broadcasting SYSTEM ARM to ALL boards (30s Unlock Window)...")
                threading.Thread(
                    target=lambda: self.send_backend_command_all("send_remote_cmd", ["arm"]),
                    daemon=True
                ).start()
            elif cmd == "/dpl_all":
                self.logger.warning("🚨 [EMERGENCY] Broadcasting FORCE PARACHUTE DEPLOY to ALL boards...")
                self._register_confirm(self.channel_ids, "dpl")
                threading.Thread(
                    target=lambda: self.send_backend_command_all("send_remote_cmd", ["dpl"]),
                    daemon=True
                ).start()
            elif cmd == "/abg_all":
                self.logger.warning("🚨 [EMERGENCY] Broadcasting AIRBAG DEPLOY to ALL boards...")
                self._register_confirm(self.channel_ids, "abg")
                threading.Thread(
                    target=lambda: self.send_backend_command_all("send_remote_cmd", ["abg"]),
                    daemon=True
                ).start()
            elif cmd == "/cal":
                self._send_recal(broadcast=False)
            elif cmd == "/cal_all":
                self._send_recal(broadcast=True)
            elif cmd == "/setch":
                # 換 LoRa 頻道。⚠ 火箭換完後,地面 dongle 也必須跟著換,
                # 否則該板立刻失聯——所以這裡只送指令並大聲提醒,不自動改本地。
                if len(parts) < 2 or not parts[1].isdigit() or not (0 <= int(parts[1]) <= 80):
                    self.logger.error("Usage: /setch <0-80>   例:/setch 72 → 922.125 MHz")
                else:
                    ch = int(parts[1])
                    freq = 850.125 + ch
                    # 合規提示(不阻擋):NCC LP0002 只准 920-925MHz = CH70~74
                    if not (70 <= ch <= 74):
                        self.logger.error(
                            f"⚠ [SETCH] CH{ch} = {freq:.3f} MHz 落在 920-925 MHz 合規頻段外!"
                            f"台灣 LP0002 只准 CH70~74(920.125~924.125 MHz)。"
                            f"指令仍會送出——請確認這是你要的。")
                    self.logger.warning(
                        f"📻 [SETCH] 要求焦點板 {self.focus_channel} 換到 CH{ch} ({freq:.3f} MHz)。"
                        f"⚠ 火箭換頻後,地面 dongle 必須也設成 CH{ch},否則此板立即失聯。"
                        f"僅 IDLE 受理;飛行中火箭會拒收。")
                    self.broadcast_event(f"[CMD] SETCH {ch}", "#00B0FF")
                    threading.Thread(
                        target=lambda: self.send_backend_command(
                            "send_remote_cmd", [f"setch_{ch}"]),
                        daemon=True).start()
            elif cmd == "/focus":
                if len(parts) >= 2 and parts[1] in self.channel_ids:
                    self.set_focus_channel(parts[1])
                else:
                    self.logger.error(f"Usage: /focus <{' | '.join(self.channel_ids)}>")
            elif cmd == "/help":
                help_msg = (
                    "Available terminal commands (must start with '/'):\n"
                    "  /port <PORT>      - Switch serial port (e.g. /port COM4)\n"
                    "  /baud <BAUDRATE>  - Switch baudrate (e.g. /baud 115200)\n"
                    "  /connect          - Start/Reconnect serial communication\n"
                    "  /disconnect       - Stop serial communication\n"
                    "  /reset-angle      - Reset IMU angle deviation\n"
                    "  /reset-data       - Reset session data (archive old CSV/raw log, start new file & reset UI)\n"
                    "  /arm              - Remote Safety ARM (focus board, 30s window)\n"
                    "  /dpl              - Emergency Force Parachute Deploy (focus board)\n"
                    "  /abg              - Emergency Deploy Airbag (focus board)\n"
                    "  /arm_all          - ARM ALL boards at once (ch1+ch2 hot-standby)\n"
                    "  /dpl_all          - Force Parachute Deploy on ALL boards at once\n"
                    "  /abg_all          - Deploy Airbag on ALL boards at once\n"
                    "  /cal              - Rocket baro re-zero + ground angle reset (focus board, IDLE only)\n"
                    "  /cal_all          - Rocket baro re-zero on ALL boards + ground angle reset\n"
                    "  /setch <0-80>     - Change rocket LoRa channel (850.125+ch MHz, IDLE only;\n"
                    "                      ground dongle MUST be changed to match or board is lost)\n"
                    "  /focus <ch>       - Switch GUI focus channel (charts/map/stage re-render)\n"
                    "Status extras: 黃燈=Format Error(資料流入但解析全失敗);"
                    "簡版狀態列 🔓ARMxx s=該板火箭回讀的解鎖倒數;"
                    "pyro 指令 10s 未見下行確認(MSG/stage)會 LOUD 告警"
                )
                self.logger.info(help_msg)
            else:
                self.logger.error(f"Unknown terminal command: {cmd}")
        else:
            self.logger.error(f"Unknown command: {text}. All commands must start with '/' (e.g. /arm, /dpl, /abg). Type /help for help.")

    def reset_gui_state(self):
        """重置 GUI 相關狀態與 UI 視覺化元件 (清空圖表、地圖、階段列表與遙測統計)"""
        self.start_time = time.time()
        self.latest_data = None
        self.last_valid_location = None
        self.last_valid_location_time = None
        
        self.est_pitch = 0.0
        self.est_roll = 0.0
        self.est_yaw = 180.0
        self.angle_deviation = 0.0
        self.max_total_accel = 0.0
        self.max_deviation_angle = 0.0
        self.max_height = 0.0
        self.calib_q = None
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.gyro_history = []
        
        self.prev_health = {}
        self.ch_latest_alt = {}   # 非焦點高度快取一併清(活頻道 0.5s 內自動回填)
        self.ch_last_fmt_err = {}
        self.ch_armed_until = {}
        self.pending_confirms = {}
        self.ch_pyro_confirmed = {}
        self.ch_view_state = {}   # 各頻道保存的統計/姿態一併清(這是真正的「重來」)
        self.ch_pyro_volt = {}
        self._prev_pyro_flags = {}
        self.ch_prev_stage = {}
        # F3-A 覆疊曲線一併清:start_time 已重置,舊 x 基準的點續留會錯位
        for ov in getattr(self, "alt_overlays", {}).values():
            ov["kh"].reset()
            ov["vz"].reset()

        # 重置 3D 姿態繪製器
        self.attitude_displayer.update(self.quaternion)
        self.ui.gl_label.setText("當前偏角: 0.0° | 最大偏角: 0.0°")

        # 重置折線圖標題與數據
        self.ui.chart_label_1.setText("高度與速度")
        self.ui.chart_label_2.setText("加速度")
        self.ui.chart_label_3.setText("姿態與角速度")
        self.chart_1.clear()
        self.chart_2.clear()
        self.chart_3.clear()

        # 重置任務階段列表 displayer
        self.stage_display.reset()

        # 重置 Leaflet 地圖 displayer
        self.location_displayer.reset()
        self.ui.map_label.setText('No Fix (No location data)')

        # 重置模組健康狀態標籤
        health_map = [
            (self.ui.health_bmp, "BMP"),
            (self.ui.health_imu, "IMU"),
            (self.ui.health_lora, "LoRa"),
            (self.ui.health_sd, "SD"),
        ]
        for lbl, name in health_map:
            lbl.setStyleSheet("background-color: rgb(150, 200, 150); color: black; border-radius: 4px; padding: 2px;")
            lbl.setText(f"{name}: OK")

        self.logger.info("UI state and visualization components have been completely reset.")



    def _add_curve_checkboxes(self, layout, chart, curve_labels: list, default_visible: list):
        """在指定 layout 中動態插入每條曲線的勾選框，插入在 Auto 勾選框之前。"""
        # 找到 Auto 勾選框的位置（layout 的最後一個 widget）
        insert_pos = layout.count() - 1
        for i, label in enumerate(curve_labels):
            cb = QCheckBox(label)
            cb.setChecked(default_visible[i])
            # 使用預設參數捕捉 i 與 chart，避免閉包陷阱
            cb.stateChanged.connect(
                lambda state, idx=i, ch=chart: ch.set_curve_visible(idx, state == 2)
            )
            layout.insertWidget(insert_pos + i, cb)

    def init_gui(self):
        self.ui.version_label.setText("v1.0.5")
        cfg = self.channel_configs.get(self.focus_channel, {})
        port = cfg.get("port", "N/A")
        baud = cfg.get("baud", "N/A")
        self._refresh_detail_label(self.focus_channel, baud)
        # 更新圖表標題
        self.ui.chart_label_1.setText("高度與速度")
        self.ui.chart_label_2.setText("加速度")
        self.ui.chart_label_3.setText("姿態與角速度")
        # Auto 捲動開關預設啟用
        self.ui.chart_checkBox_1.setChecked(True)
        self.ui.chart_checkBox_2.setChecked(True)
        self.ui.chart_checkBox_3.setChecked(True)
        self.ui.map_checkBox.setChecked(True)
        self.ui.gl_label.setText("當前偏角: 0.0° | 最大偏角: 0.0°")

        # 動態插入 [同步 X 軸] 勾選框
        self.sync_chart_cb = QCheckBox("同步 X 軸")
        self.sync_chart_cb.setChecked(True)
        def toggle_sync(state):
            sync = (state == 2)
            self.chart_2.set_x_link(self.chart_1 if sync else None)
            self.chart_3.set_x_link(self.chart_1 if sync else None)
        self.sync_chart_cb.stateChanged.connect(toggle_sync)
        self.ui.horizontalLayout_5.addWidget(self.sync_chart_cb)
        toggle_sync(2)

        # ── F2/F5:焦點切換+火工品操作列;原生 4 顆 Auto 收進全域「Auto 跟隨」──
        # 原 checkbox 隱藏不移除:update_ui 仍讀它們(繪圖邏輯零改動),
        # 由 global_auto_cb 代理設定。
        self._build_pyro_button_row()
        for cb in (self.ui.chart_checkBox_1, self.ui.chart_checkBox_2,
                   self.ui.chart_checkBox_3, self.ui.map_checkBox):
            cb.hide()

        # 動態插入各圖表的曲線勾選框 (暫時隱藏，因為使用者可以直接操作圖例)
        # self._add_curve_checkboxes(
        #     self.ui.horizontalLayout_5, self.chart_1,
        #     ['KH', 'RH', 'VZ'],
        #     [True, True, True]
        # )
        # self._add_curve_checkboxes(
        #     self.ui.horizontalLayout_7, self.chart_2,
        #     ['GA', 'AX', 'AY', 'AZ'],
        #     [True, True, False, False]  # 預設只顯示 GA 和 AX
        # )
        # self._add_curve_checkboxes(
        #     self.ui.horizontalLayout_8, self.chart_3,
        #     ['Pitch', 'Roll', 'GX', 'GY', 'GZ'],
        #     [True, True, False, False, False]  # 預設只顯示姿態角
        # )
        # 初始化時同步非預設可見的曲線狀態
        self.chart_2.set_curve_visible(2, False)  # AY
        self.chart_2.set_curve_visible(3, False)  # AZ
        self.chart_3.set_curve_visible(3, False)  # GX
        self.chart_3.set_curve_visible(4, False)  # GY
        self.chart_3.set_curve_visible(5, False)  # GZ

        self.ui.listWidget.clear()
        
    def _get_mapped_axis(self, data, key):
        """將 SensorData 的 raw 屬性依據 axis_config 對應轉換並套用正負號"""
        config_val = self.axis_config.get(key, f"+{key}")
        sign = -1.0 if config_val.startswith("-") else 1.0
        var_name = config_val.lstrip("+-")
        val = getattr(data, var_name, 0.0)
        return sign * val

    def handle_angle_change(self, pitch: float, roll: float, yaw: float):
        # euler_to_quaternion 參數對應：第一參數繞 Y 軸 (自旋/縱向), 第二參數繞 X 軸 (俯仰), 第三參數繞 Z 軸 (側向)
        # 1. 繞 Y 軸縱向自旋 (Roll / self-spin)
        spin_q = euler_to_quaternion(roll, 0, 0)
        # 2. 繞 X 軸橫向俯仰 (Pitch)
        pitch_q = euler_to_quaternion(0, pitch, 0)
        # 3. 繞 Z 軸側向傾斜 (Yaw-tilt)
        yaw_q = euler_to_quaternion(0, 0, yaw)
        
        # 組合旋轉：先自旋 ➔ 再俯仰 ➔ 最後套用側向偏航傾斜
        q_temp = quaternion_multiply(pitch_q, spin_q)
        quaternion = quaternion_multiply(yaw_q, q_temp)
        quaternion = quaternion / np.linalg.norm(quaternion)

        return quaternion

    def get_deviation_angle(self, q1, q2):
        """計算兩個四元數代表的縱向 Y 軸方向向量之間的 3D 偏航夾角 (度)"""
        if q1 is None or q2 is None:
            return 0.0
        # 縱向指向在相對於四元數旋轉後的單位向量公式為 R * [0, 1, 0]^T
        # 即旋轉矩陣 R 的第二列 (Y列):
        # vx = 2 * (x*y - w*z)
        # vy = 1 - 2*(x*x + z*z)
        # vz = 2 * (y*z + w*x)
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        # 確保單位四元數
        n1 = math.sqrt(w1*w1 + x1*x1 + y1*y1 + z1*z1)
        if n1 > 0:
            w1, x1, y1, z1 = w1/n1, x1/n1, y1/n1, z1/n1
        n2 = math.sqrt(w2*w2 + x2*x2 + y2*y2 + z2*z2)
        if n2 > 0:
            w2, x2, y2, z2 = w2/n2, x2/n2, y2/n2, z2/n2

        v1x = 2.0 * (x1 * y1 - w1 * z1)
        v1y = 1.0 - 2.0 * (x1 * x1 + z1 * z1)
        v1z = 2.0 * (y1 * z1 + w1 * x1)

        v2x = 2.0 * (x2 * y2 - w2 * z2)
        v2y = 1.0 - 2.0 * (x2 * x2 + z2 * z2)
        v2z = 2.0 * (y2 * z2 + w2 * x2)

        # 點積求夾角
        dot = v1x * v2x + v1y * v2y + v1z * v2z
        dot = max(-1.0, min(1.0, dot))
        return math.degrees(math.acos(dot))

    def broadcast_event(self, label_text: str, color: str = "#D500F9"):
        """在三張折線圖與 GPS 地圖上同步繪製事件標記線/卡片"""
        if self.latest_data:
            x_val = self.latest_data.gs_timestamp - self.start_time
        else:
            x_val = time.time() - self.start_time

        time_str = datetime.now().strftime("%H:%M:%S")
        full_label = f"[{time_str}] {label_text}"

        # 縮寫映射表：圖表只顯示簡短縮寫，不顯示時間戳
        _ABBR_MAP = {
            "[CMD] ARM":          "ARM",
            "[CMD] DPL":          "DPL",
            "[CMD] ABG":          "ABG",
            "[CMD] Reset Angle":  "RST",
            "[IGNITION]":         "IGN",
            "[BURNOUT]":          "BRN",
            "[APOGEE]":           "APG",
            "[PARACHUTE_DEPLOY]": "DPL",
            "[TOUCHDOWN]":        "TDN",
            "[AIRBAG_DEPLOY]":    "ABG",
        }
        # 找縮寫；MSG 類型取 MSG 前綴；否則截取 [] 內容最多 4 字
        chart_label = _ABBR_MAP.get(label_text)
        if chart_label is None:
            if label_text.startswith("[MSG]"):
                chart_label = "MSG"
            else:
                # 通用後備：取第一個 [] 內的縮寫（最多 4 個字元）
                m = re.search(r'\[([^\]]+)\]', label_text)
                chart_label = m.group(1)[:4] if m else label_text[:4]

        self.chart_1.add_event_marker(x_val, chart_label, color)
        self.chart_2.add_event_marker(x_val, chart_label, color)
        self.chart_3.add_event_marker(x_val, chart_label, color)

        if self.latest_data and self.latest_data.location:
            self.location_displayer.add_event_marker(self.latest_data.location, full_label, color)

        self.logger.info(f"[EVENT BROADCAST] Marked event: {full_label}")


    def update_ui(self, data: SensorData):
        has_fix = False
        if data.gnss_state:
            has_fix = "FIX" in data.gnss_state.upper() and "NO_FIX" not in data.gnss_state.upper()
        else:
            # 舊版 JSON 資料相容性檢查 (非預設值即視為有定位)
            has_fix = data.location != (25.0, 121.5) and data.location != (23.5, 121.5)

        if has_fix:
            self.last_valid_location = data.location
            self.last_valid_location_time = data.timestamp
            time_str = self.last_valid_location_time.strftime("%H:%M:%S")
            self.ui.map_label.setText(f'Latitude:{round(data.location[0],5)} | Longitude:{round(data.location[1],5)} (Locked, {time_str})')
            # 座標與軌跡線永遠更新；Auto 勾選框只控制鏡頭是否自動跟隨
            self.location_displayer.update(data.location, follow=self.ui.map_checkBox.isChecked(), time_str=time_str)
        else:
            if self.last_valid_location:
                time_str = self.last_valid_location_time.strftime("%H:%M:%S")
                self.ui.map_label.setText(
                    f'Latitude:{round(self.last_valid_location[0],5)} | Longitude:{round(self.last_valid_location[1],5)} '
                    f'(Lost Lock - Last Update: {time_str})'
                )
            else:
                self.ui.map_label.setText('No Fix (No location data)')

        # 依軸向對應讀取並映射感測器數據，同時扣除靜止校準得到的陀螺儀零點偏置
        ax = self._get_mapped_axis(data, "ax")
        ay = self._get_mapped_axis(data, "ay")
        az = self._get_mapped_axis(data, "az")
        gx = self._get_mapped_axis(data, "gx") - self.gyro_bias_x
        gy = self._get_mapped_axis(data, "gy") - self.gyro_bias_y
        gz = self._get_mapped_axis(data, "gz") - self.gyro_bias_z

        # 基於映射後的對地重力向量計算 Roll / Pitch
        try:
            roll_rad = math.atan2(ay, az)
            pitch_rad = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            body_roll_acc = roll_rad * 180.0 / math.pi
            body_pitch_acc = pitch_rad * 180.0 / math.pi
        except Exception:
            body_roll_acc = 0.0
            body_pitch_acc = 0.0

        # 計算 dt
        dt = 0.1
        if self.latest_data:
            dt = (data.timestamp - self.latest_data.timestamp).total_seconds()
            if data.timestamp_ms and self.latest_data.timestamp_ms:
                dt = (data.timestamp_ms - self.latest_data.timestamp_ms) / 1000.0
            # 限制合理區間以防通訊中斷造成數值暴增
            if dt <= 0 or dt > 1.0:
                dt = 0.1

        # 歐拉角姿態融合 (自適應互補濾波)
        # 計算總加速度大小 (單位為 g)
        total_acc = math.sqrt(ax**2 + ay**2 + az**2)
        acc_deviation = abs(total_acc - 1.0)
        
        # 動態調整互補濾波權重：當運動產生額外加速度時，降低對加速度計的信任，依靠陀螺儀積分
        if acc_deviation < 0.08:
            alpha = 0.05
        elif acc_deviation > 0.25:
            alpha = 0.0
        else:
            alpha = 0.05 * (1.0 - (acc_deviation - 0.08) / 0.17)
        
        # 如果是首幀，直接將估算值對齊感測器讀值
        if not self.latest_data:
            self.est_pitch = body_roll_acc
            self.est_roll = -body_pitch_acc
            self.est_yaw = 180 - ((data.direction - self.angle_deviation + 360) % 360)
            self.calib_q = self.handle_angle_change(self.est_pitch, self.est_yaw, self.est_roll)
            self.max_deviation_angle = 0.0
            self.max_total_accel = data.total_accel
            self.max_height = data.kfh_height
        else:
            # 1. Pitch 估算 (對應橫向俯仰)：整合 X 軸陀螺儀 (gx) 並以加速度計 Roll 修正
            self.est_pitch = (1 - alpha) * (self.est_pitch + gx * dt) + alpha * body_roll_acc
            
            # 2. Roll 估算 (對應側向傾斜)：整合 Y 軸陀螺儀 (gy) 並以加速度計 Pitch 修正
            self.est_roll = (1 - alpha) * (self.est_roll - gy * dt) - alpha * body_pitch_acc
            
            # 3. Yaw 估算 (對應縱向自旋)：整合 Z 軸陀螺儀 (gz)，若有有效航向則以 target_yaw 修正
            target_yaw = 180 - ((data.direction - self.angle_deviation + 360) % 360)
            if data.direction != 0.0:
                self.est_yaw = (1 - alpha) * (self.est_yaw + gz * dt) + alpha * target_yaw
            else:
                self.est_yaw = (self.est_yaw + gz * dt) % 360

        # 💡 使用地面站接收的高精度相對時間軸 X
        x_val = data.gs_timestamp - self.start_time

        # Chart 1:主曲線已移除(雙頻道對照圖,曲線由 overlay 畫);
        # 仍呼叫 update([]) 推進時間軸與 auto_scroll(X 捲動+sync-X 的錨點)
        self.chart_1.update(
            [],
            auto_scroll=self.ui.chart_checkBox_1.isChecked(),
            x_value=x_val
        )
        # Chart 2：合加速度（GA）與三軸加速度（AX, AY, AZ）
        self.chart_2.update(
            [data.total_accel, data.ax, data.ay, data.az],
            auto_scroll=self.ui.chart_checkBox_2.isChecked(),
            x_value=x_val
        )
        # Chart 3：姿態角（Pitch, Roll, Yaw）與角速度（GX, GY, GZ）
        self.chart_3.update(
            [self.est_pitch, self.est_yaw - 180.0, self.est_roll, data.gx, data.gy, data.gz],
            auto_scroll=self.ui.chart_checkBox_3.isChecked(),
            x_value=x_val
        )

        self.quaternion = self.handle_angle_change(self.est_pitch, self.est_yaw, self.est_roll)
        self.attitude_displayer.update(self.quaternion)

        if self.calib_q is None:
            self.calib_q = self.quaternion

        self.max_total_accel = max(self.max_total_accel, data.total_accel)
        self.max_height = max(self.max_height, data.kfh_height)
        current_dev = self.get_deviation_angle(self.quaternion, self.calib_q)
        self.max_deviation_angle = max(self.max_deviation_angle, current_dev)

        # 動態更新圖表上方標籤顯示具體數值(F3-B:尾端併排非焦點頻道最新高度;
        # 5 秒沒新資料顯示 "--" 防止死板舊值偽裝成活資料)
        _now = time.time()
        others = " ".join(
            (f"┃{c}: {alt:.0f}m {vz:+.1f}m/s" if _now - ts < 5.0 else f"┃{c}: --")
            for c, (alt, vz, ts) in sorted(self.ch_latest_alt.items())
            if c != self.focus_channel
        )
        self.ui.chart_label_1.setText(
            f"高度與速度 ] 箭端高度: {data.kfh_height:.1f} m | 最大高度: {self.max_height:.1f} m | 垂直速度: {data.vz:.1f} m/s {others}"
        )
        self.ui.chart_label_2.setText(
            f"加速度 ] 當前總加速度: {data.total_accel:.2f} g | 最大總加速度: {self.max_total_accel:.2f} g"
        )
        self.ui.chart_label_3.setText(
            f"姿態與角速度 ] Pitch: {self.est_pitch:.1f}° | Roll: {self.est_roll:.1f}° | Yaw: {(self.est_yaw - 180.0):.1f}°"
        )
        self.ui.gl_label.setText(
            f"當前偏角: {current_dev:.1f}° | 最大偏角: {self.max_deviation_angle:.1f}°"
        )

        is_new_event, ev_name, ev_color = self.stage_display.update(data.stage, data.timestamp)
        if is_new_event:
            self.broadcast_event(f"[{ev_name}]", ev_color)
        # Update health status labels based on failedTasks (0:BMP, 1:IMU, 2:LoRa, 3:SD)
        health_map = [
            (self.ui.health_bmp, "BMP"),
            (self.ui.health_imu, "IMU"),
            (self.ui.health_lora, "LoRa"),
            (self.ui.health_sd, "SD"),
        ]
        for idx, (lbl, name) in enumerate(health_map):
            is_failed = idx in data.failedTasks
            was_failed = self.prev_health.get(idx, False)
            
            if is_failed != was_failed:
                if is_failed:
                    self.logger.warning(f"[HEALTH] Module '{name}' status changed: OK -> FAIL")
                else:
                    self.logger.info(f"[HEALTH] Module '{name}' status changed: FAIL -> OK")
                self.prev_health[idx] = is_failed
            
            if is_failed:
                lbl.setStyleSheet("background-color: rgb(180, 70, 70); color: white; border-radius: 4px; padding: 2px;")
                lbl.setText(f"{name}: FAIL")
            else:
                lbl.setStyleSheet("background-color: rgb(150, 200, 150); color: black; border-radius: 4px; padding: 2px;")
                lbl.setText(f"{name}: OK")
        self.latest_data = data

    def update_ui_from_zmq(self, topic: str, data: SensorData):
        """ZMQ 資料接收槽，會更新心跳時間戳並視焦點分發"""
        prev_status = self.channel_status.get(topic, "No Data")
        self.last_recv_time[topic] = time.time()
        self.channel_status[topic] = "Connected"

        if prev_status in ["No Data", "Lost", "Stale", "Backend Offline"]:
            self.logger.info(f"Telemetry channel '{topic}' connection established/resumed.")

        # R1 第二證據源:stage「轉入」DEPLOYING(2)/DEPLOYED(3) 的那一刻。
        # ★必須是邊緣(轉入)而非位準(現在是 2/3):stage 一旦轉開傘就整趟飛行
        #   維持不變,用位準判斷會讓「之後」送出的任何開傘指令在 0.5s 內被
        #   這個舊狀態判定為成功——上行完全死掉也照樣顯示 ✅,確認機制在最
        #   需要它誠實的時候失效。
        # ★prev_stage is None(該頻道第一筆)不觸發:GUI 中途啟動時火箭可能
        #   早就開傘了,那不構成「剛剛送出的指令被收到」的證據。
        # (火箭現行 5 態;st.md 12 態上機後此對映要改——12 態的 2=上升段)
        prev_stage = self.ch_prev_stage.get(topic)
        self.ch_prev_stage[topic] = data.stage
        if prev_stage is not None and data.stage in (2, 3) and prev_stage not in (2, 3):
            self._confirm_pyro(topic, "dpl", "STAGE")

        # pyro 電源監測(火箭 PB1/PB0 ADC)
        self._track_pyro_power(topic, data)

        # 收包瞬間該通道 LED 亮綠脈衝(下一輪 check_heartbeats 200ms 內覆蓋回狀態色)。
        # Backend Offline 狀態不閃:命令路徑疑似死亡時不給「健康綠」的假象。
        led = self.ch_leds.get(topic)
        if led and self.channel_status.get(topic) != "Backend Offline":
            led.setStyleSheet("background-color: #00FF00; border-radius: 6px; border: 1px solid #00AA00;")

        # F3-A:chart1=雙頻道對照圖 —— 「所有」頻道的 KH/VZ 都推進各自的
        # 覆疊曲線(焦點實線已刪,頻道平權全時段畫;時間基準同 chart 群)
        ov = self.alt_overlays.get(topic)
        if ov:
            x = data.gs_timestamp - self.start_time
            ov["kh"].push(x, data.kfh_height)
            ov["vz"].push(x, data.vz)

        if topic == self.focus_channel:
            self.update_ui(data)
        else:
            # 非焦點頻道:記錄最新高度供 chart1 標題併排顯示(F3-B 數字欄)。
            # 帶時間戳:渲染端 5s 過期改顯 "--",死板的舊高度不得偽裝成活資料。
            self.ch_latest_alt[topic] = (data.kfh_height, data.vz, time.time())

    def _is_backend_running(self, focus_ch: str) -> bool:
        """透過本機 TCP 探針檢測後端 Daemon (ZMQ CMD/PUB Port) 是否運作中"""
        cfg = self.channel_configs.get(focus_ch, {})
        zmq_cmd_port = cfg.get("zmq_cmd_port")
        zmq_port = cfg.get("zmq_port")
        ports_to_check = [p for p in (zmq_cmd_port, zmq_port) if p]
        if not ports_to_check:
            return False

        for port in ports_to_check:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.settimeout(0.05)
                    if s.connect_ex(("127.0.0.1", port)) == 0:
                        return True
            except Exception:
                pass
        return False

    # LED 色票(沿用 jx06 五態規則;集中定義供逐通道渲染)
    _LED_CSS = "background-color: {c}; border-radius: 6px;{b}"

    def _set_led(self, ch: str, color: str, border: str = "", tooltip: str = ""):
        led = self.ch_leds.get(ch)
        if led:
            b = f" border: 1px solid {border};" if border else ""
            led.setStyleSheet(self._LED_CSS.format(c=color, b=b))
            if tooltip:
                led.setToolTip(f"{ch}: {tooltip}")

    def _backend_online_cached(self, ch: str, now: float) -> bool:
        """backend TCP 探針加 1s 快取:兩通道 x 5Hz 全探會放大 GUI 卡頓風險"""
        if not hasattr(self, "_probe_cache"):
            self._probe_cache = {}
        ts, val = self._probe_cache.get(ch, (0.0, False))
        if now - ts >= 1.0:
            val = self._is_backend_running(ch)
            self._probe_cache[ch] = (now, val)
        return val

    def check_heartbeats(self):
        """定期 (5Hz) 檢查「所有」通道心跳:逐通道刷新 LED+右下簡版狀態;
        焦點通道另外寫 serial_label 詳細字串(維持 jx06 原版格式)。"""
        now = time.time()

        for ch in self.channel_ids:
            last_time = self.last_recv_time.get(ch)
            cfg = self.channel_configs.get(ch, {})
            port = cfg.get("port", "N/A")
            baud = cfg.get("baud", "N/A")
            prev_status = self.channel_status.get(ch)
            is_focus = (ch == self.focus_channel)

            # ★資料證據優先:1.5s 內有遙測=後端 PUB 側必然活著,TCP 探針的
            #   偶發 false-negative(GUI 卡頓漏 50ms timeout/探針快取窗)不得
            #   壓過它——否則 log 每 200ms 洗「offline↔resumed」+LED 綠紫頻閃。
            has_fresh_data = (last_time is not None and now - last_time < 1.5)
            # 格式錯誤證據:backend 5s 內回報過解析失敗=資料在流(也證明後端活著)
            fmt_recent = (ch in self.ch_last_fmt_err
                          and now - self.ch_last_fmt_err[ch] < 5.0)
            if not self._backend_online_cached(ch, now) and not has_fresh_data \
                    and not fmt_recent:
                short = f"{ch} {port} ✖後端未啟動"
                color = "#9933FF" if int(now * 2) % 2 == 0 else "#442266"
                self._set_led(ch, color, "#6600CC", "Backend Offline(紫=後端服務未啟動)")
                self.channel_status[ch] = "Backend Offline"
                if prev_status != "Backend Offline":
                    self.logger.warning(f"Telemetry backend daemon for channel '{ch}' is offline! "
                                        f"Please start main.py or run_persist_backend.bat. (紫燈=此狀態)")
                if is_focus:
                    self._refresh_detail_label(ch, baud)
                self._set_port_label(ch, short, "#B366FF")
                continue

            # ★格式錯誤黃燈:資料狂流但每行解析都失敗≠「無資料」——插錯裝置/
            #   鮑率錯/對到別人訊號時,橙色 No Data 會把人帶去查天線(錯方向)。
            #   有 fresh 遙測時不搶(偶發壞行免驚擾)。
            if fmt_recent and not has_fresh_data:
                status_txt = "Format Error (資料流入但解析全失敗)"
                color = "#FFD600" if int(now * 2) % 2 == 0 else "#665500"
                self._set_led(ch, color, "#AA8800",
                              "Format Error(黃=有資料流但看不懂:查裝置/鮑率/頻段)")
                self.channel_status[ch] = "Format Error"
                if prev_status != "Format Error":
                    self.logger.error(
                        f"⚠ Channel '{ch}' receiving data but ALL lines fail to parse! "
                        f"Wrong device on {port}? Wrong baudrate? Foreign signal? (黃燈=此狀態)")
                self._set_port_label(ch, f"{ch} {port} ⚠格式錯", "#FFDD44")
                if is_focus:
                    self._refresh_detail_label(ch, baud)
                continue

            if last_time is None:
                status_txt = "No Data (後端已連線/待資料)"
                self._set_led(ch, "#FF6600", "#CC3300", "No Data(後端在跑、還沒收到遙測)")
                self.channel_status[ch] = "No Data"
                self._set_port_label(ch, f"{ch} {port} ◌無資料", "#FF9955")
            else:
                elapsed = now - last_time
                if elapsed < 1.5:
                    status_txt = f"Connected ({elapsed:.1f}s ago)"
                    self._set_led(ch, "#00CC00", "#00AA00", f"Connected({elapsed:.1f}s)")
                    self.channel_status[ch] = "Connected"
                    self._set_port_label(ch, f"{ch} {port} ✔{elapsed:.1f}s", "#66DD66")
                elif elapsed < 5.0:
                    status_txt = f"Stale ({elapsed:.1f}s ago)"
                    color = "#FFA500" if int(now * 5) % 2 == 0 else "#555555"
                    self._set_led(ch, color, "#CC8400", f"Stale({elapsed:.1f}s)")
                    self.channel_status[ch] = "Stale"
                    if prev_status == "Connected":
                        self.logger.warning(f"Telemetry channel '{ch}' connection stale. "
                                            f"Last data received {elapsed:.1f}s ago.")
                    self._set_port_label(ch, f"{ch} {port} ⚠{elapsed:.1f}s", "#FFB84D")
                else:
                    status_txt = f"Telemetry Lost ({elapsed:.1f}s ago)"
                    self._set_led(ch, "#FF0000", "#AA0000", f"Lost({elapsed:.0f}s)")
                    self.channel_status[ch] = "Lost"
                    if prev_status in ["Connected", "Stale"]:
                        self.logger.error(f"Telemetry channel '{ch}' RF lost! "
                                          f"No data received for {elapsed:.1f}s.")
                    self._set_port_label(ch, f"{ch} {port} ✖{elapsed:.0f}s", "#FF6666")

            if is_focus:
                self._refresh_detail_label(ch, baud)

        # ── R1:pyro 指令逾時未見下行確認 → LOUD 告警(開環變閉環的另一半)──
        for key in [k for k, p in self.pending_confirms.items() if now > p["deadline"]]:
            pch, paction = key
            del self.pending_confirms[key]
            self.logger.error(
                f"🔴 [UNCONFIRMED] {pch} {paction.upper()} 已送出但 10s 內未見下行確認"
                f"(MSG SUCCESS / stage 轉開傘皆無)——火箭可能沒收到,或遙測中斷。"
                f"確認另一板狀態後考慮重發;勿因單板未確認而中止(冗餘設計)。")
            self.broadcast_event(f"[🔴 {pch} {paction.upper()} 未確認]", "#FF3B30")

    def _refresh_detail_label(self, ch: str, baud):
        """右側詳細列。★與左側簡版整合(2026-07-26):簡版已經逐頻道顯示
        「COM 號 + 連線狀態 + pyro 電壓」,詳細版再印一次 port/status 純屬重複、
        還把有限的橫向空間吃掉。改成只補簡版沒有的東西——焦點板的硬體識別
        (VID/PID,插錯 dongle 一眼看穿)與連線參數。"""
        port = self.channel_configs.get(ch, {}).get("port", "N/A")
        hw = self._port_hw_info(port)
        # 焦點標記 ▶ 只放在左側簡版一次,這裡不再重複(先前兩邊都印 ▶chN)
        self.ui.serial_label.setText(f"{ch}｜{hw or '未知裝置'}｜{baud} 8N1")
        self.ui.serial_label.setToolTip(
            f"{ch} = {port}\n"
            + (f"硬體:{hw}" if hw else "查不到 VID/PID:裝置未插上,或驅動未提供識別碼"))

    def _set_port_label(self, ch: str, text: str, color: str):
        lbl = self.ch_port_labels.get(ch)
        if lbl:
            # R4:火箭端回讀的 ARMED 視窗倒數直接掛在簡版狀態列(每板獨立)
            armed_left = self.ch_armed_until.get(ch, 0) - time.time()
            if armed_left > 0:
                text += f" 🔓ARM{armed_left:.0f}s"
            # 焦點板加箭頭:單板指令(/dpl、/arm…)打到哪塊板一眼可辨
            if ch == self.focus_channel:
                text = "▶" + text
            # pyro 電源狀態(10s 內的量測才顯示,過期不留舊值誤導)
            pv = self.ch_pyro_volt.get(ch)
            if pv and time.time() - pv[2] < 10.0:
                vf, va = pv[0], pv[1]
                if 0 <= vf < self._PYRO_LIVE_V:
                    text += " 🔴熔斷"
                elif va >= self._PYRO_LIVE_V:
                    text += f" 🔓武裝{va:.1f}V"
                elif vf >= self._PYRO_LIVE_V:
                    text += f" 🔒{vf:.1f}V"
            lbl.setText(text)
            lbl.setStyleSheet(f"color: {color};")
            hw = self._port_hw_info(self.channel_configs.get(ch, {}).get("port"))
            lbl.setToolTip(hw or "無硬體識別資訊(裝置未插或 VID/PID 不可讀)")

    def _port_hw_info(self, port) -> str:
        """查 COM 埠的 VID/PID+產品名(30s 快取)。回傳如
        'FT232R USB UART [0403:6001]',查不到回 None——用於分辨
        「哪個 dongle 是哪個頻段」,COM 號換插孔就變、VID/PID 不會。"""
        now = time.time()
        if now - self._ports_scan_ts > 30.0:
            try:
                import serial.tools.list_ports as _lp
                self._ports_cache = {p.device.lower(): p for p in _lp.comports()}
            except Exception:
                self._ports_cache = {}
            self._ports_scan_ts = now
        p = self._ports_cache.get(str(port or "").lower())
        if not p:
            return None
        vidpid = (f"{p.vid:04X}:{p.pid:04X}"
                  if p.vid is not None and p.pid is not None else "????")
        name = (p.product or p.description or "").strip()
        return f"{name} [{vidpid}]" if name else f"[{vidpid}]"


    # ── R1/R4:火箭下行 MSG 事件的結構化解析(閉環的下行半邊)──────
    _MSG_RE = re.compile(r"^🚀 \[ROCKET MSG\] \[(\w+)\] (.*)$")
    _MSG_COLORS = {"SUCCESS": "#00C853", "WARN": "#FF9100", "WARNING": "#FF9100",
                   "ERROR": "#FF3B30", "ERR": "#FF3B30", "FAIL": "#FF3B30"}

    def _handle_rocket_msg(self, ch: str, message: str):
        """解析火箭 MSG 事件:ARMED 回讀(R4)、pyro 下行確認(R1)、圖表標記。
        burst 4 連發會重複命中——確認/解除都冪等,標記只在首次觸發。"""
        m = self._MSG_RE.match(message)
        if not m:
            return
        level, content = m.group(1).upper(), m.group(2)

        if "MANUAL ARMED" in content:
            mm = re.search(r"within (\d+)s", content)
            secs = int(mm.group(1)) if mm else 30
            already = self.ch_armed_until.get(ch, 0) > time.time()
            self.ch_armed_until[ch] = time.time() + secs
            if not already:
                self.logger.warning(f"🔓 [{ch}] Rocket confirms ARMED ({secs}s window)")
                self.broadcast_event(f"[🔓 {ch} ARMED]", "#FF9100")
        elif "MANUAL SAFE" in content or "ARM expired" in content:
            self.ch_armed_until.pop(ch, None)
        elif "Parachute deployed successfully" in content:
            self._confirm_pyro(ch, "dpl", "MSG")
        elif "Airbag inflation started" in content:
            self._confirm_pyro(ch, "abg", "MSG")
        elif "REJECT" in content:
            # 指令被火箭閘門拒收:等待確認中的板要立刻知道,不必空等 10s。
            # ★必須比對「被拒的是哪一道指令」:舊版只比對頻道,結果一句
            #   RECAL/SETCH 的拒收會把待確認的開傘指令一起清掉,而且把紅色
            #   告警貼上「DPL 被拒收」的錯誤標籤——操作員會以為開傘指令失敗。
            low = content.lower()
            if "already deployed" in low:
                # 語意是「傘已經開了」= 開傘的證據,不是失敗。burst 第 2~4 發
                # 必然收到這句;SUCCESS 幀掉包時這是唯一的線索,不可誤報成紅色失敗。
                self._confirm_pyro(ch, "dpl", "MSG(already deployed)")
                return
            # 韌體的 pyro 拒收會自報 dpl/abg;非 pyro 指令(recal/setch/channel)
            # 一律不動 pending——它們跟火工品無關。
            acts = [a for a in ("dpl", "abg") if a in low]
            if not acts:
                if any(t in low for t in ("recal", "setch", "channel")):
                    self.logger.warning(f"⚠ [{ch}] 非火工品指令被拒收:{content}")
                    return
                # 舊韌體的拒收訊息不帶指令名(向後相容):只能全清,但講清楚
                acts = ["dpl", "abg"]
                self.logger.warning(
                    f"⚠ [{ch}] 火箭回報拒收但未指明指令(舊版韌體):{content}")
            hit = [k for k in self.pending_confirms if k[0] == ch and k[1] in acts]
            for k in hit:
                del self.pending_confirms[k]
                self.logger.error(f"🚫 [{ch}] {k[1].upper()} REJECTED by rocket: {content}")
            if hit:
                self.broadcast_event(f"[🚫 {ch} REJECT]", "#FF3B30")
        elif level in ("ERROR", "ERR", "FAIL"):
            self.broadcast_event(f"[{ch}] {content[:36]}", self._MSG_COLORS["ERROR"])

    # pyro 電源判讀門檻(2S 鋰電 7.4V 標稱 / 8.4V 滿電)
    _PYRO_LIVE_V = 5.0    # 高於此 = 該段線路確實帶電
    _PYRO_LOW_V  = 6.6    # 帶電但低於此 = 電池偏低,發射前該換

    def _track_pyro_power(self, ch: str, data):
        """追蹤火箭下行的 pyro 電源電壓,狀態翻轉時發告警。
        VF(保險絲後端):掉到 0V = 熔斷。誤觸發時電流走 safety shunt 燒斷保險絲
          ——點火頭沒被點著(人安全),但整條 pyro 電源同時死亡,不修就上天 = 傘開不了。
        VA(arming 開關後端):>0V = 已武裝,這是規範 4.6.7 要求的「遠端驗證啟動狀態」。
        -1 = 該板韌體沒有這個功能(舊版),不做任何判讀。"""
        vf = getattr(data, "v_fuse", -1.0)
        va = getattr(data, "v_arm", -1.0)
        if vf < 0 and va < 0:
            return
        self.ch_pyro_volt[ch] = (vf, va, time.time())

        blown = (0 <= vf < self._PYRO_LIVE_V)
        armed = (va >= self._PYRO_LIVE_V)
        prev = self._prev_pyro_flags.get(ch)
        if prev == (blown, armed):
            return
        self._prev_pyro_flags[ch] = (blown, armed)

        if prev is not None and blown != prev[0]:
            if blown:
                self.logger.error(
                    f"🔴 [{ch}] PYRO FUSE BLOWN — 保險絲後端 {vf:.2f}V。"
                    f"誤觸發已被並聯導線擋下(點火頭未點著),但這塊板現在「點不了火」。"
                    f"發射前必須更換保險絲;另一板若正常仍可獨立完成回收。")
                self.broadcast_event(f"[🔴 {ch} 保險絲熔斷]", "#FF3B30")
            else:
                self.logger.info(f"✅ [{ch}] pyro 電源恢復 {vf:.2f}V(保險絲已更換)")
        if prev is not None and armed != prev[1]:
            if armed:
                self.logger.warning(f"🔓 [{ch}] PYRO ARMED — arming 開關已導通 {va:.2f}V"
                                    f"(儲能裝置進入啟動狀態,人員勿靠近火箭)")
                self.broadcast_event(f"[🔓 {ch} 已武裝]", "#FF9100")
            else:
                self.logger.info(f"🔒 [{ch}] pyro 已解除武裝(arming 開關斷開)")
        if not blown and 0 <= vf < self._PYRO_LOW_V:
            self.logger.warning(f"⚠ [{ch}] pyro 電池偏低 {vf:.2f}V(2S 標稱 7.4V)")

    def _confirm_pyro(self, ch: str, action: str, src: str, evidence_at: float = None):
        """R1:收到火箭下行的點火證據(MSG SUCCESS 或 stage 轉入開傘)。
        evidence_at = 這份證據產生的時刻;早於指令送出時刻的證據不予採信。"""
        ev = time.time() if evidence_at is None else evidence_at
        key = (ch, action)
        first = key not in self.ch_pyro_confirmed
        self.ch_pyro_confirmed[key] = ev
        pend = self.pending_confirms.get(key)
        if pend is not None and ev >= pend["sent_at"]:
            del self.pending_confirms[key]
            self.logger.info(f"✅ [CONFIRMED] {ch} {action.upper()} verified via {src} downlink")
            self.broadcast_event(f"[✅ {ch} {action.upper()} OK]", "#00C853")
        elif first:
            # 沒按過鈕卻收到開傘證據=自動開傘(正常飛行)或另一位操作員,標記即可
            self.broadcast_event(f"[✅ {ch} {action.upper()}]", "#00C853")

    def _register_confirm(self, chs, action: str):
        """R1:pyro 指令送出時登記「等待下行確認」,10s 未見證據 → LOUD 告警。
        ★記下送出時刻:確認只能由「這次指令之後」出現的證據滿足。否則備援
          指令會被上一次開傘留下的舊狀態瞬間「確認」,即使上行早已中斷。"""
        now = time.time()
        for c in chs:
            self.pending_confirms[(c, action)] = {"sent_at": now, "deadline": now + 10.0}

    def poll_zmq_data(self):
        """非阻塞讀取 ZMQ 消息，保證 UI 流暢不被卡死"""
        while True:
            try:
                # 採用 zmq.NOBLOCK，若無新消息會立刻拋出 zmq.Again 異常並 break 結束
                topic_bytes, payload_bytes = self.zmq_socket.recv_multipart(flags=zmq.NOBLOCK)
                topic = topic_bytes.decode('utf-8')
                payload_dict = json.loads(payload_bytes.decode('utf-8'))
                
                if topic.endswith("_log"):
                    # 💡 本地接收背景連線/重試日誌並分發，使其自動呈現於 GUI Log 視窗中
                    level_str = payload_dict.get("level", "INFO")
                    message = payload_dict.get("message", "")
                    logger_name = payload_dict.get("logger", "backend")
                    level = getattr(logging, level_str, logging.INFO)
                    logging.getLogger(logger_name).log(level, message)

                    ch = topic[:-4]   # "ch1_log" -> "ch1"
                    # 格式錯誤黃燈證據:communicator 每行解析失敗都 logger.error 這個前綴
                    if message.startswith("Format error:"):
                        self.ch_last_fmt_err[ch] = time.time()
                    # 火箭 MSG 事件(communicator 已改寫成 🚀 前綴;
                    # 舊版 startswith("MSG ") 判斷永遠不命中=死碼,已由此取代)
                    elif message.startswith("🚀 [ROCKET MSG]"):
                        self._handle_rocket_msg(ch, message)
                    continue

                sensor_data = SensorData.from_dict(payload_dict)
                self.update_ui_from_zmq(topic, sensor_data)
            except zmq.Again:
                break
            except Exception as e:
                self.logger.error(f"Error polling ZMQ message: {e}")
                break

    def closeEvent(self, event):
        """視窗關閉時釋放 ZMQ 資源"""
        self.logger.info("MainWindow close event detected. Releasing ZMQ context and sockets...")
        try:
            self.zmq_poll_timer.stop()
            self.zmq_socket.close()
            self.zmq_context.term()
        except Exception as e:
            self.logger.error(f"Error terminating ZMQ connection on exit: {e}")
        event.accept()

if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow(["ch1"])
    window.show()
    app.exec()