import numpy as np
import logging
import threading
import math
import socket
import re
from collections import deque
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
from src.utils.settings import load_axis_up, load_channel_settings, save_channel_settings

class MainWindow(QMainWindow):
    def __init__(self, channel_ids=None):
        super().__init__()
        self.logger = logging.getLogger(__name__)
        self.angle_deviation = 0.0
        self.max_total_accel = 0.0
        self.max_deviation_angle = 0.0
        self.max_height = 0.0
        self._kh_prev = None   # KH 突波過濾：三點中位數用
        self._kh_prev2 = None
        self._seen_boost = False   # 推導 BURNOUT 用：確定看過推力段才算數
        self._seen_descent = False # 推導 TOUCHDOWN 用：確定看過下降段才算數
        self._ign_cand = None      # 推導 IGNITION 用：離架確認前不定案
        self._ign_above = False
        self.calib_q = None
        
        # ─── 快速軸向對應設定區 (Sensor Axis Mapping Configuration) ───
        # 定義：[火箭本體標準軸向] ➔ [感測器原始軸向] (支援正負號，例如 "-ay"、"+ax" 等)
        # 標準本體定義：Z_body=縱向自旋軸, X_body=橫向俯仰軸, Y_body=橫向偏航軸
        #
        # ★2026-07-31 航電板改成【豎放】。姿態算式（update_ui）寫死了
        #   roll = atan2(ay, az)、pitch = atan2(-ax, hypot(ay,az))
        #   —— 它假設【az 對齊重力】。板子一豎起來 az≈0，atan2 落在奇異點上：
        #   火箭在發射台站得筆直，畫面卻顯示 ±90°，而 roll 會被 az 的雜訊
        #   放大成亂跳。偏角(angle_deviation)是相對 calib_q 算的，受害較小，
        #   但姿態視窗與 Pitch/Roll 曲線會完全不能看。
        #
        #   下面六組是「哪一支感測器軸朝上」各自對應的重映射。全部都是
        #   【右手座標】—— 只把兩軸對調而不補負號會翻轉手性，陀螺儀的
        #   旋轉方向會整個反過來，比不改還糟。六組的行列式都是 +1，
        #   有回歸測試守著（tests/test_axis_mapping.py）。
        #
        # ★2026-08-01 實測定案：AXIS_UP = "-x"，不自動偵測。
        #   把板子從平放立起來的那一刻，AX 從 0 掉到 −1.0 g 並停在那裡
        #   （加速度圖上是一個乾淨的台階，GA 全程維持 1.00 g）。
        #   加速度計量的是重力的反作用力、指向【上】，所以讀到 AX=−1
        #   代表朝上的是感測器的 −X 軸。
        #
        #   本來做了發射台自動偵測，實測後拿掉：這個值量一次就定了，
        #   多一層「開機後六秒內會自己改設定」的機制，等於在發射前多一個
        #   會動的東西 —— 而它動錯的時候沒有人看得出來（姿態圖本來就
        #   不直觀）。寫死的常數看得見、改得動、也不會自己變。
        #   要臨時改用 /axis <dir>，或在 settings.json 放 "axis"。
        self.AXIS_PRESETS = {
            #        ax     ay     az     gx     gy     gz
            "+z": ("+ax", "+ay", "+az", "+gx", "+gy", "+gz"),  # 平放（原本的假設）
            "+x": ("+ay", "+az", "+ax", "+gy", "+gz", "+gx"),  # 循環
            "+y": ("+az", "+ax", "+ay", "+gz", "+gx", "+gy"),  # 循環
            "-z": ("+ay", "+ax", "-az", "+gy", "+gx", "-gz"),
            "-x": ("+az", "+ay", "-ax", "+gz", "+gy", "-gx"),
            "-y": ("+ax", "+az", "-ay", "+gx", "+gz", "-gy"),
        }
        self._AXIS_KEYS = ("ax", "ay", "az", "gx", "gy", "gz")

        # 目前這枚火箭的安裝方向（豎放，感測器 −X 朝上）。實測值，見上方說明。
        self.AXIS_UP_DEFAULT = "-x"
        # settings.json 的 "axis" 可覆蓋（例如兩塊板裝的方向不同）。
        self.axis_up = load_axis_up() or self.AXIS_UP_DEFAULT
        self.axis_config = dict(zip(self._AXIS_KEYS, self.AXIS_PRESETS[self.axis_up]))
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
        self.ch_seq = {}              # ch -> 最近收到的序號(deque):量化掉包率
        self.ch_link = {}             # ch -> (到達率 0~1, 樣本數)
        self._prev_batt_level = {}
        self._prev_drift_level = {}   # ch -> 氣壓基準漂移等級,只在惡化時告警    # ch -> "ok"/"low"/"crit":只在惡化時警告

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

        self._init_placeholder_labels()

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
        # ★2026-07-31：LINGER=0。
        # 預設 LINGER 是 -1（無限），意思是 close() 之後 context.term() 會
        # 【一直等到排隊中的訊息送出去為止】。而下面 finally 裡就是
        # close() + term()：只要後端沒回應（recv 逾時走 zmq.error.Again），
        # 那筆已送出但沒被取走的請求就永遠在佇列裡 → term() 永久阻塞。
        #
        # 這些呼叫跑在背景執行緒（/dpl 用 threading.Thread 發），所以畫面
        # 不會凍住 —— 它只是每逾時一次就洩漏一條卡死的執行緒和一個
        # ZMQ context（各自帶一條 I/O 執行緒）。發射台上後端沒起來、
        # COM 拔錯孔的時候，一連按幾次就累積起來了。
        # LINGER=0 = 關掉時直接丟棄未送出的訊息，term() 立刻返回。
        # 對這裡是正確語意：那筆請求已經逾時了，補送也沒有意義。
        socket.setsockopt(zmq.LINGER, 0)
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
        #    雙板熱備援,任一板成功開傘即可安全著陸)→ 提示補點,不是中止。
        #    ★2026-07-31:氣囊移除後這句話的份量更重了——傘是唯一的減速手段,
        #    沒有第二層。所以「補點失敗的那塊板」現在是必做,不是選做。 ──
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
        # 單板點火鈕(每板:傘=dpl;走該板 backend 單發)
        # ★2026-07-31:氣囊移除,「囊」鈕一併拿掉。
        #   火箭端 PA0 已併入降落傘發火迴路(傘要 PA0+PA1 同時驅動),
        #   單獨送 abg 只會半驅動傘迴路——點不著,卻讓操作員以為做了事。
        #   在只剩一種火工品的情況下留著一顆會被拒收的紅鈕,是在緊急時
        #   多給操作員一個按錯的選項。
        for ch in self.channel_ids:
            row.addWidget(self._make_pyro_button(f"傘 {ch}", ch, "dpl"))

        row.addWidget(self._vsep())
        # 廣播鈕(兩板同時;沿用 _all 的並行+LOUD 部分失敗告警)
        row.addWidget(self._make_pyro_button("傘 ALL", None, "dpl"))

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

        # ── 新增: 隱藏 Port 失敗與重試日誌核取方塊 ──
        self.hide_port_err_cb = QCheckBox("隱藏 Port 重試")
        self.hide_port_err_cb.setChecked(False)
        self.hide_port_err_cb.toggled.connect(self._on_hide_port_err_toggled)
        row.addWidget(self.hide_port_err_cb)

    # ★2026-08-01：Qt Designer 的預留字。
    #
    # ui_main.py 裡有 11 個 QLabel 的初值是 "TextLabel"，那是 Designer 產生的
    # 佔位符。它們【只在有遙測時】才會被覆寫，所以在還沒收到資料的機器上
    # （或 backend 沒起來時）就一直是 "TextLabel"。
    #
    # 而 health_* 那四個更麻煩：唯一會設定它們的另一條路是 reset_gui_state()，
    # 那裡把四個全部設成【綠色 OK】—— /reset-data 之後畫面說 BMP/IMU/LoRa/SD
    # 都正常，但一筆資料都還沒進來。憑空宣稱的綠燈比 "TextLabel" 危險得多，
    # 一併改成灰色的「—」（未知）。
    _HEALTH_UNKNOWN_QSS = ("background-color: rgb(200, 200, 200); color: rgb(90, 90, 90); "
                           "border-radius: 4px; padding: 2px;")

    def _init_placeholder_labels(self):
        """把 .ui 的 "TextLabel" 換成有意義的初值（資料進來前就該看得懂）"""
        self.ui.gl_label.setText("當前偏角: --  |  最大偏角: --")
        self.ui.map_label.setText("等待 GPS…")
        self.ui.chart_label_1.setText("高度與速度")
        self.ui.chart_label_2.setText("加速度")
        self.ui.chart_label_3.setText("姿態與角速度")
        self.ui.serial_label.setText("尚未連線")
        for lbl, name in ((self.ui.health_bmp, "BMP"), (self.ui.health_imu, "IMU"),
                          (self.ui.health_lora, "LoRa"), (self.ui.health_sd, "SD")):
            lbl.setStyleSheet(self._HEALTH_UNKNOWN_QSS)
            lbl.setText(f"{name}: —")
        # version_label 由既有邏輯填（v1.0.5），這裡只保證不是 "TextLabel"
        if self.ui.version_label.text() == "TextLabel":
            self.ui.version_label.setText("")

    def _on_hide_port_err_toggled(self, checked: bool):
        if hasattr(self, 'log_display') and self.log_display:
            self.log_display.set_hide_port_errors(checked)

    _KH_SPIKE_M = 8.0        # KH 比【前後兩包都】高出這麼多 = 孤立突波
                             # 真實頂點附近 0.5s 只變 1.2m，不會誤判
    _IGN_GA_THR = 1.5        # 推力起來的門檻（g）
    _IGN_MAX_LEAD_S = 5.0    # 點火→離架偵測的合理上限；超過不採信候選

    def _hide_retry_flight_guard(self, data):
        """★2026-08-01：一離架就強制解除折疊。

        發射台上折疊是對的（ch2 重試 331 次會把畫面洗掉），但飛行中
        序列埠掉線是【必須立刻看到】的事 —— 那代表遙測正在消失，
        而遙測是唯一可靠的飛行資料（SD 已實測 reset 鎖卡救不回來）。

        只做一次，之後操作員仍可自行再勾起來（他要是真的想）。"""
        if getattr(data, "stage", 0) < 1:
            return
        if getattr(self, "_hide_retry_released", False):
            return
        self._hide_retry_released = True
        cb = getattr(self, "hide_port_err_cb", None)
        if cb is not None and cb.isChecked():
            cb.setChecked(False)
            self.logger.warning("🔊 已離架 —— 自動解除「隱藏 Port 重試」，"
                                "飛行中掉線必須看得到")

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
        self._kh_prev = None   # KH 突波過濾：三點中位數用
        self._kh_prev2 = None
        self._seen_boost = False   # 推導 BURNOUT 用：確定看過推力段才算數
        self._seen_descent = False # 推導 TOUCHDOWN 用：確定看過下降段才算數
        self._ign_cand = None      # 推導 IGNITION 用：離架確認前不定案
        self._ign_above = False
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
            elif cmd in ("/hide_retry", "/hideretry"):
                if not args:
                    new_state = not self.hide_port_err_cb.isChecked()
                else:
                    arg = args[0].lower()
                    new_state = arg in ("on", "true", "1")
                self.hide_port_err_cb.setChecked(new_state)
                status_str = "ENABLED" if new_state else "DISABLED"
                self.logger.info(f"Hiding port retry logs is now {status_str}")
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
                if self._ascent_guard([self.focus_channel], "dpl"):
                    return
                self.logger.warning("🚨 [EMERGENCY] Transmitting remote FORCE PARACHUTE DEPLOYMENT command...")
                self.broadcast_event("[CMD] DPL", "#D500F9")
                self._register_confirm([self.focus_channel], "dpl")
                threading.Thread(
                    target=lambda: self.send_backend_command("send_remote_cmd", ["dpl"]),
                    daemon=True
                ).start()
            elif cmd in ("/abg", "/abg_all"):
                # ★2026-07-31:氣囊移除。指令保留成明確的錯誤訊息而不是刪掉,
                # 是因為肌肉記憶——這兩道指令練了整個賽前,緊急時手指會自己打。
                # 打了什麼都沒發生(Unknown command)比打了被告知去路更糟。
                self.logger.error(
                    "🚫 氣囊已於 2026-07-31 移除,/abg 與 /abg_all 不再有作用。"
                    "PA0 現在是降落傘發火迴路的一半——要開傘請用 /dpl 或 /dpl_all。")
            # ── 雙板廣播:同時對所有航電板(ch1+ch2 熱備援)發命令 ──
            elif cmd == "/arm_all":
                self.logger.warning("🚨 [SAFETY] Broadcasting SYSTEM ARM to ALL boards (30s Unlock Window)...")
                threading.Thread(
                    target=lambda: self.send_backend_command_all("send_remote_cmd", ["arm"]),
                    daemon=True
                ).start()
            elif cmd == "/dpl_all":
                if self._ascent_guard(self.channel_ids, "dpl"):
                    return
                self.logger.warning("🚨 [EMERGENCY] Broadcasting FORCE PARACHUTE DEPLOY to ALL boards...")
                self._register_confirm(self.channel_ids, "dpl")
                threading.Thread(
                    target=lambda: self.send_backend_command_all("send_remote_cmd", ["dpl"]),
                    daemon=True
                ).start()
            elif cmd == "/gndtest":
                if len(parts) >= 2 and parts[1].lower() in ("off", "0", "stop"):
                    self.logger.warning("🧪 [GNDTEST] 送出【關閉】地面測試模式。")
                    self.broadcast_event("[🧪 GNDTEST OFF]", "#7e8a9b")
                    threading.Thread(
                        target=lambda: self.send_backend_command(
                            "send_remote_cmd", ["gndtest_off"]),
                        daemon=True).start()
                    return
                # 規範 4.5.3 的地面測試入口。解除 PB6 手動發火的閘門，
                # 10 分鐘後韌體自動恢復 —— 忘了關也不會帶上發射台。
                # PB6 仍需 IDLE + ARM 才會動作，所以這一道本身不會點火。
                self.logger.warning(
                    "🧪 [GNDTEST] 送出地面測試模式（10 分鐘後自動失效）。"
                    "PB6 手動發火在 IDLE + ARM 之下才會動作 —— "
                    "⚠ 確認火工品已斷開再按 PB6。")
                self.broadcast_event("[🧪 GNDTEST]", "#FFD600")
                threading.Thread(
                    target=lambda: self.send_backend_command("send_remote_cmd", ["gndtest"]),
                    daemon=True).start()
            elif cmd == "/axis":
                if len(parts) >= 2 and parts[1] in self.AXIS_PRESETS:
                    up = parts[1]
                    self.axis_config = dict(zip(self._AXIS_KEYS, self.AXIS_PRESETS[up]))
                    self.axis_up = up
                    self.logger.warning(
                        f"🧭 軸向改為 {up} → {self.axis_config}。"
                        f"這只影響本次執行；要永久生效請在 settings.json 加 \"axis\"。")
                    self.broadcast_event(f"[🧭 軸向 {up}]", "#00B0FF")
                else:
                    self.logger.info(
                        f"🧭 目前軸向：{self.axis_up}"
                        f"（預設 {self.AXIS_UP_DEFAULT}，2026-08-01 實測：豎放時 AX≈−1g）\n"
                        f"   {self.axis_config}\n"
                        f"   用法：/axis <+z|+x|+y|-z|-x|-y>\n"
                        f"   意義：火箭站直時，哪一支【感測器】軸朝上"
                        f"（加速度計量重力反作用力，朝上那支讀 +1g）")
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
                # 指令依【性質】分組，不是依字母。緊急時要找的是「開傘」，
                # 不是「以 d 開頭的那個」。🔴 標的是會點火工品的。
                help_msg = "\n".join([
                    "指令一覽（全部以 / 開頭）",
                    "",
                    "🔴 火工品 —— 會真的點火，動作前確認目標板",
                    "   /arm              解鎖焦點板（30 秒窗口）",
                    "   /arm_all          解鎖【全部】板（雙板熱備援）",
                    "   /dpl              強制開傘 · 焦點板",
                    "   /dpl_all          強制開傘 · 【全部】板",
                    "                     上升中（stage=1 且 vz>2）會要求再輸入一次",
                    "",
                    "🧪 地面測試",
                    "   /gndtest          開啟地面測試模式（10 分鐘後自動失效）",
                    "                     解除 PB6 手動發火閘門；PB6 仍需 IDLE + ARM",
                    "   /gndtest off      立刻關閉，不必等逾時",
                    "",
                    "🎯 校準 —— 只在 IDLE 受理，飛行中火箭會拒收",
                    "   /cal              氣壓零點重校 · 焦點板 ＋ 地面姿態歸零",
                    "   /cal_all          氣壓零點重校 · 【全部】板 ＋ 地面姿態歸零",
                    "   /reset-angle      只歸零地面端的姿態偏角（不送指令給火箭）",
                    "   /axis [dir]       IMU 安裝軸向 +z|+x|+y|-z|-x|-y（預設 -x＝豎放）",
                    "",
                    "📻 連線",
                    "   /port <PORT>      切換序列埠，例：/port COM4",
                    "   /baud <RATE>      切換鮑率，例：/baud 9600",
                    "   /connect          連線／重新連線",
                    "   /disconnect       中斷連線",
                    "   /setch <0-80>     換火箭 LoRa 頻道（850.125+ch MHz，IDLE 限定）",
                    "                     ⚠ 火箭換完，地面 dongle 也要換，否則該板立刻失聯",
                    "                     ⚠ 台灣 LP0002 只准 CH70~74（920.125~924.125 MHz）",
                    "",
                    "🖥 介面／資料",
                    "   /focus <ch>       切換焦點頻道（圖表／地圖／階段跟著重畫）",
                    "   /reset-data       封存目前的 CSV 與 raw log，開新檔並清空畫面",
                    "   /hide_retry [on|off]  隱藏序列埠重試 log（也有核取方塊）",
                    "                     ⚠ 開啟後埠真的掛掉不會在 log 出聲；",
                    "                       狀態列的鏈路指示不受影響，照樣會變",
                    "   /help             這張表",
                    "",
                    "讀畫面",
                    "   黃燈              有資料流入但解析全失敗（格式不符）",
                    "   🔓ARMxx s         火箭回讀的解鎖倒數（不是地面端自己算的）",
                    "   火工品指令        送出後 10 秒內沒看到下行確認會大聲告警",
                    "   階段列 灰色「—」   該列是地面推導、目前還沒推導出來",
                    "   階段列 紅底        火箭列被跳過＝那段遙測整段掉包",
                ])
                self.logger.info(help_msg)
            else:
                self.logger.error(f"Unknown terminal command: {cmd}")
        else:
            self.logger.error(f"Unknown command: {text}. All commands must start with '/' (e.g. /arm, /dpl, /cal). Type /help for help.")

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
        self._kh_prev = None   # KH 突波過濾：三點中位數用
        self._kh_prev2 = None
        self._seen_boost = False   # 推導 BURNOUT 用：確定看過推力段才算數
        self._seen_descent = False # 推導 TOUCHDOWN 用：確定看過下降段才算數
        self._ign_cand = None      # 推導 IGNITION 用：離架確認前不定案
        self._ign_above = False
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
        self.ch_seq = {}
        self.ch_link = {}
        self._prev_batt_level = {}
        self._prev_drift_level = {}
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
        # ★2026-08-01：reset 之後是【沒有資料】，不是【一切正常】。
        # 舊碼把四個全設成綠色 OK —— /reset-data 按下去，畫面立刻宣稱
        # BMP/IMU/LoRa/SD 都健康，而那一刻一筆遙測都還沒進來。
        # 下一筆資料到達時 update_ui 會用真實狀態覆寫，在那之前顯示「未知」。
        for lbl, name in health_map:
            lbl.setStyleSheet(self._HEALTH_UNKNOWN_QSS)
            lbl.setText(f"{name}: —")

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

    # 圖上的事件縮寫。★用「關鍵字比對」而不是完全比對的對照表。
    #
    # 舊版是一張 10 筆的 dict，鍵是完整字串（"[CMD] ARM" 之類）。但事件標籤
    # 現在有 23 個產生點，大多是帶 emoji 和頻道名的 f-string ——
    # "[✅ ch1 DPL OK]"、"[🚫 ch2 REJECT]"、"[🔴 ch1 基準漂移 12m]"。
    # 那些永遠對不中，全部掉進後備的「取 [] 內前 4 字」，於是圖上一整排
    # 都是「emoji + ch」，彼此完全分不出來，而且看不出是哪一塊板。
    #
    # 依序比對、先中先贏。順序有意義：GNDTEST OFF 必須排在 GNDTEST 前面。
    _ABBR_RULES = [
        ("Reset Angle", "RST"),  ("SETCH", "SETCH"), ("CAL", "CAL"),
        ("GNDTEST OFF", "GND-"), ("GNDTEST", "GND+"),
        ("軸向", "AXIS"),        ("上升中", "!ASC"),
        ("基準漂移", "DRIFT"),   ("未確認", "!ACK"),
        ("保險絲熔斷", "FUSE"),  ("電量危險", "BATT"),
        ("已武裝", "ARMD"),      ("ARMED", "ARM"),
        ("REJECT", "REJ"),
        ("PARACHUTE_DEPLOY", "DPL"), ("DPL", "DPL"), ("ARM", "ARM"),
        ("BURNOUT", "BRN"),      ("APOGEE", "APG"),
        ("TOUCHDOWN", "TDN"),    ("LAUNCH", "LNCH"),
        ("DESCENT", "DESC"),     ("LANDED", "LAND"),
        ("MSG", "MSG"),
    ]

    def _chart_abbr(self, label_text: str) -> str:
        """事件標籤 → 圖表上的短縮寫（含頻道號，才分得出是哪塊板）"""
        inner = label_text
        m = re.search(r"\[([^\]]+)\]", label_text)
        if m:
            # "[CMD] DPL" 這種：[] 之後還有內容，要一起看
            inner = (m.group(1) + " " + label_text[m.end():]).strip()
        ch = ""
        mc = re.search(r"\bch(\d+)", inner)
        if mc:
            ch = mc.group(1)
        ok = "✓" if "OK" in inner else ""
        for key, abbr in self._ABBR_RULES:
            if key in inner:
                if abbr == "SETCH":
                    mn = re.search(r"SETCH\s*(\d+)", inner)
                    return f"CH{mn.group(1)}" if mn else "SETCH"
                return f"{abbr}{ok}{ch}"
        # 真的認不出來：去掉 emoji 與頻道名再截斷，至少留下可讀的字
        rest = re.sub(r"[^\w一-鿿]+", "", re.sub(r"\bch\d+\b", "", inner))
        return (rest[:5] or "EVT") + ch

    def broadcast_event(self, label_text: str, color: str = "#D500F9",
                        src_data=None):
        """在三張折線圖與 GPS 地圖上同步繪製事件標記線/卡片。

        src_data = 觸發這個事件的那一幀遙測。★由遙測推導出來的事件一定要傳，
        操作員手打的指令不用（那時「現在」才是對的）。

        ★2026-08-01：不傳的話會用 self.latest_data —— 而 update_ui 是在
          【最後】才 self.latest_data = data，所有推導都跑在那之前。
          結果是每一個事件標記都畫在【上一幀】，比觸發它的資料早整整
          一個遙測週期（2Hz → 0.5 秒）。

          畫面上看得很清楚：APOGEE 標在高度峰值左邊 0.5 秒，而開傘標記
          反而落在峰值上。事後拿這些標記論證開傘時序時，每一個事件都
          系統性早半秒 —— 而 C 備援的最小餘裕只有 1.69 秒。
        """
        d = src_data if src_data is not None else self.latest_data
        if d is not None:
            x_val = d.gs_timestamp - self.start_time
        else:
            x_val = time.time() - self.start_time

        time_str = datetime.now().strftime("%H:%M:%S")
        full_label = f"[{time_str}] {label_text}"

        chart_label = self._chart_abbr(label_text)

        self.chart_1.add_event_marker(x_val, chart_label, color)
        self.chart_2.add_event_marker(x_val, chart_label, color)
        self.chart_3.add_event_marker(x_val, chart_label, color)

        # ★2026-08-01：這裡是台北那個假標記的來源。
        # 舊碼只檢查 location 是否為真值，而無定位時它是 (25.0, 121.5)
        # ——一個永遠為真的假座標。現在 models 會給 None，這個判斷就對了；
        # 但仍明確比對 gnss_state，免得日後有人又把預設值加回去。
        if d is not None and d.location and getattr(d, "gnss_state", "") != "NO_FIX":
            self.location_displayer.add_event_marker(d.location, full_label, color)
        elif d:
            # 不靜默跳過：事後對照事件與位置時，要知道這個事件本來就沒有座標
            self.logger.info(f"[EVENT] {full_label} 無 GPS 定位，未在地圖上標記")

        self.logger.info(f"[EVENT BROADCAST] Marked event: {full_label}")


    def update_ui(self, data: SensorData):
        has_fix = False
        if data.gnss_state:
            has_fix = "FIX" in data.gnss_state.upper() and "NO_FIX" not in data.gnss_state.upper()
        else:
            # 舊版 JSON 相容：沒有 gnss_state 欄位時，有座標就當作有定位。
            # ★2026-08-01：不再比對 (25.0,121.5)/(23.5,121.5) 這兩個魔術值
            # —— models 已經不會產生它們了，留著只會讓人以為那還是有效的判斷。
            has_fix = data.location is not None

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

        # ★2026-08-01：max_height 擋掉單包突波（三點中位數式）。
        #
        # 實測會出現【單包 KH 比鄰近值高 10m 以上】的突波。原本是
        #     self.max_height = max(self.max_height, data.kfh_height)
        # 一包就把頂點永久灌高 —— 而那個值會進 APOGEE 標籤、進畫面，
        # 也是報告書上要交的頂點高度。
        #
        # 判準是【比前後兩包都高出 _KH_SPIKE_M】= 孤立高點。
        #   真實爬升：單調上升，永遠不會比後一包高 → 不受影響
        #   真實頂點：自由落體在頂點附近 0.5s 只變 1.2m → 遠低於門檻
        #   突波：比前後都高 10m 以上 → 擋掉
        # 代價是 max_height 落後一包（0.5s），頂點附近高度幾乎不變，看不出來。
        #
        # ⚠ 第一版寫成「要連續兩包背書、取 min」是錯的 —— 真實爬升時每兩包
        #   只採信較舊的那一個，130 m/s 上升會少報上百公尺，比突波還糟。
        #
        # ⚠ 韌體端沒有對應保護：rel_alt 完全沒有突波抑制，peak_rel_alt 直接
        #   吃 `if (rel_alt > peak) peak = rel_alt`。開傘受的影響見
        #   doc/open_defects_20260801.md。這裡只擋住【顯示與報告】。
        kh = data.kfh_height
        if self._kh_prev is not None:
            pv = self._kh_prev
            is_spike = (self._kh_prev2 is not None
                        and pv - self._kh_prev2 > self._KH_SPIKE_M
                        and pv - kh > self._KH_SPIKE_M)
            if is_spike:
                self.logger.warning(
                    f"⚠ KH 單包突波 {pv:.1f}m（前後 {self._kh_prev2:.1f} / "
                    f"{kh:.1f}m）—— 未計入最大高度。氣壓突波也會重置開傘"
                    f"條件 B 的 1.5 秒計時，見 open_defects_20260801.md")
            else:
                self.max_height = max(self.max_height, pv)
        self._kh_prev2 = self._kh_prev
        self._kh_prev = kh

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

        # ★把火箭自己的 uptime 一起傳進去 —— 飛行時間軸要用火箭的時鐘，
        # 不是地面站收到封包的牆上時間（見 stage_display._rel）。
        is_new_event, ev_name, ev_color = self.stage_display.update(
            data.stage, data.timestamp, getattr(data, "timestamp_ms", None))
        if is_new_event:
            self.broadcast_event(f"[{ev_name}]", ev_color, data)

        # ── 地面站自行推導的事件（火箭端的 5 個狀態刻意不含這些）──────────
        # 資料本來就在遙測裡，推導出來標成「地面推導」與火箭回報視覺上分開。
        # 火箭端不加狀態的理由：那 5 個是開傘決策實際在用的，多一個就要
        # 重新稽核所有 flight_state 比較，賽前不值得動。
        # ★2026-08-01：原本 6 個推導列裡只有 BURNOUT 與 APOGEE 有人點亮，
        # 其餘 4 列（ARMED / IGNITION / COASTING / TOUCHDOWN）永遠顯示「—」。
        # 一直是「—」的列看起來像壞掉，而且會讓真正沒推導出來的情況失去意義。
        # 這四個的資料本來就在遙測裡，補上。
        ms = getattr(data, "timestamp_ms", None)
        ga = getattr(data, "total_accel", float("nan"))

        def derive(name, color):
            if self.stage_display.mark_derived(name, data.timestamp, color, ms):
                self.broadcast_event(f"[{name}]", color, data)

        if data.stage == 0:
            # IGNITION 的【候選】—— ★不在這裡定案。
            #
            # 舊碼是「ST:0 期間第一個 GA>1.5 就標 IGNITION」，那是錯的：
            # ST:0 是整段發射台待命時間，搬上架、放下、有人撞到、陣風，
            # 任何一次超過 1.5g 都會標下去。而 mark_derived 是一次性的 ——
            # 誤標之後【真正的點火再也蓋不掉】。
            #
            # 改成：記住最後一次「由下往上穿過門檻」的時刻，一直覆寫；
            # 等 ST 真的變成 1（火箭自己確認離架）才把它定案。
            # 推力段 GA 會持續在門檻之上，所以離架前的最後一個上升沿
            # 就是點火。搬動造成的尖峰會被後來真正的點火覆寫掉。
            if not math.isnan(ga):
                if ga >= self._IGN_GA_THR and not self._ign_above:
                    self._ign_cand = (data.timestamp, ms)   # 上升沿
                    self._ign_above = True
                elif ga < self._IGN_GA_THR:
                    self._ign_above = False
        elif data.stage == 1:
            # ★離架確認了 —— 現在才把點火候選定案。
            # 距離太遠的候選不採信：點火到「2.5g 持續 200ms」約 1.3 秒，
            # 給到 5 秒已經很寬。超過就是搬動留下的舊尖峰，或真正的
            # 上升沿掉包了 —— 那時寧可讓 IGNITION 維持「—」，
            # 也不要標一個編出來的時間。
            if self._ign_cand is not None:
                c_ts, c_ms = self._ign_cand
                if c_ms and ms:
                    lead = (ms - c_ms) / 1000.0
                else:
                    lead = (data.timestamp - c_ts).total_seconds()
                if 0 <= lead <= self._IGN_MAX_LEAD_S:
                    if self.stage_display.mark_derived("IGNITION", c_ts,
                                                       "#00E676", c_ms):
                        self.broadcast_event("[IGNITION]", "#00E676", data)
                self._ign_cand = None
            # BURNOUT：合加速度跌回 1.15g 以下 = 推力結束（與韌體同一門檻）
            if not math.isnan(ga) and ga < 1.15 and getattr(self, "_seen_boost", False):
                derive("BURNOUT", "#FF9100")
                # COASTING：推力結束的同一刻慣性上升就開始了。時間戳相同不是
                # 冗餘 —— BURNOUT 是「推力沒了」，COASTING 是「開始純靠慣性」，
                # 報告書上兩者要分開講。
                derive("COASTING", "#FFC400")
            if ga > 1.5:
                self._seen_boost = True
        elif data.stage == 2:
            # APOGEE：進入開傘的那一刻，峰值已經確定
            derive(f"APOGEE {self.max_height:.0f}m", "#FFD600")
        elif data.stage == 3:
            # TOUCHDOWN：觸地／觸水的瞬間。★這不等於 LANDED ——
            # 韌體的 LANDED 要「靜止 10 秒」才成立，也就是【觸水後 10 秒】。
            # 落海時真正要記下來的位置是觸水點，不是十秒後漂走的地方。
            vz = getattr(data, "vz", 0.0)
            if abs(vz) > 3.0:
                self._seen_descent = True
            elif getattr(self, "_seen_descent", False) and abs(vz) < 1.0:
                derive("TOUCHDOWN", "#00E5FF")
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
        # ★2026-08-01：折疊解除的守衛必須在【焦點分發之前】。
        # 放在 update_ui 裡是錯的 —— 那個函式只對焦點頻道跑，而操作員
        # 會去勾這個框，正是因為某個頻道在洗版；如果焦點就停在那個
        # 掛掉的頻道上，它一幀都不會進來，解除永遠不觸發。
        # 掛在這裡則是【任一頻道】離架就解除，符合「同一枚火箭」的事實。
        self._hide_retry_flight_guard(data)

        prev_stage = self.ch_prev_stage.get(topic)
        self.ch_prev_stage[topic] = data.stage
        if prev_stage is not None and data.stage in (2, 3) and prev_stage not in (2, 3):
            self._confirm_pyro(topic, "dpl", "STAGE")

        # pyro 電源監測(火箭 PB1/PB0 ADC)
        self._track_pyro_power(topic, data)
        # 鏈路品質:用火箭的發送序號跳號量化掉包
        self._track_link_quality(topic, data)
        # 氣壓基準漂移:地面上 RH 應該 ≈0,不是 0 就是 ref_press 過期了
        self._track_baro_drift(topic, data)

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
            # ★2026-08-01：分壓電路沒焊，VF/VA 是浮接雜訊（見 _track_pyro_power）。
            #   保留讀值收集（ch_pyro_volt 仍會更新，焊好後只要把 if False 改掉），
            #   但不再顯示 —— 永遠掛著的「🔴熔斷」會讓真的熔斷看不出來。
            pv = self.ch_pyro_volt.get(ch)
            if False and pv and time.time() - pv[2] < 10.0:   # ← 焊好後移除 False and
                vf, va = pv[0], pv[1]
                if 0 <= vf < self._PYRO_LIVE_V:
                    text += " 🔴熔斷"
                elif va >= self._PYRO_LIVE_V:
                    text += f" 🔓武裝{va:.1f}V"
                elif vf >= self._PYRO_LIVE_V:
                    lvl = self._batt_level(vf)
                    icon = {"ok": "🔋", "low": "🪫", "crit": "🪫⚠"}.get(lvl, "🔒")
                    text += f" {icon}{vf:.1f}V"
            # 鏈路到達率(靠火箭序號跳號算出來的,不是訊號強度)
            lt = self._link_text(ch)
            if lt:
                text += " " + lt[0]
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
                # ★階段序列的 ARMED 那一列：這是火箭【回讀】的解鎖確認，
                # 不是地面站送出指令的時刻 —— 規範 4.6.7 要的就是回讀。
                # 只認第一塊回報的板（mark_derived 自帶去重）。
                self.stage_display.mark_derived(
                    "ARMED", datetime.now(), "#FF9100",
                    getattr(self.latest_data, "timestamp_ms", None))
        elif "MANUAL SAFE" in content or "ARM expired" in content:
            self.ch_armed_until.pop(ch, None)
        elif "Parachute deployed successfully" in content:
            self._confirm_pyro(ch, "dpl", "MSG")
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
            # "abg" 仍留在清單裡:韌體現在會回「REJECT abg - airbag removed」,
            # 必須被辨識成【非開傘】的拒收。拿掉的話它會掉進下面的
            # 「舊韌體未指明指令」fallback,把待確認的 dpl 一起清掉、
            # 並在畫面上誤報成開傘被拒——正是本段註解在防的那件事。
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

    # ── 鏈路品質:用火箭的發送序號量化掉包 ──────────────────────────
    _LINK_WINDOW = 40     # 滾動視窗(2Hz → 約 20 秒)
    _LINK_MIN_N  = 8      # 少於這麼多樣本不下結論(剛連上時別亂報)

    def _track_link_quality(self, ch: str, data):
        """火箭每發一包 SQ 遞增 1。地面端只要看序號跳號就知道掉了幾包——
        這是唯一能量化「RF 到達率」的資料,而且火箭本來就在數了,只是
        以前沒放進封包(CSV 的 lora_seq 因此恆為 0)。
        seq==0 表示舊韌體沒帶這欄位,直接不做統計。"""
        seq = getattr(data, "lora_seq", 0)
        if not seq:
            return
        dq = self.ch_seq.get(ch)
        if dq is None:
            dq = deque(maxlen=self._LINK_WINDOW)
            self.ch_seq[ch] = dq
        # 序號倒退 = 火箭重開機(序號歸零)→ 舊樣本作廢,重新起算
        if dq and seq < dq[-1]:
            dq.clear()
            self.logger.info(f"[{ch}] 序號重置(火箭重新開機?),掉包統計重新起算")
        dq.append(seq)
        if len(dq) >= self._LINK_MIN_N:
            span = dq[-1] - dq[0] + 1          # 這段期間火箭「應該」發了幾包
            if span > 0:
                self.ch_link[ch] = (len(dq) / span, len(dq))

    def _link_text(self, ch: str):
        """回傳 (顯示字串, 顏色);資料不足回 None。"""
        lk = self.ch_link.get(ch)
        if not lk:
            return None
        rate, n = lk
        pct = max(0.0, min(1.0, rate)) * 100.0
        color = "#66DD66" if pct >= 90 else ("#FFDD44" if pct >= 70 else "#FF6666")
        return (f"📶{pct:.0f}%", color)

    # pyro 電源判讀門檻(2S 鋰電:8.4V 滿 / 7.4V 標稱 / 6.0V 空)
    _PYRO_LIVE_V = 5.0    # 高於此 = 該段線路確實帶電
    _PYRO_LOW_V  = 7.0    # 低於此 = 電量偏低(約剩 20%),發射前應更換
    _PYRO_CRIT_V = 6.6    # 低於此 = 電量危險(約剩 5%),不應繼續飛

    # ── 氣壓基準漂移門檻(公尺)。換算:1 hPa ≈ 8.3 m ─────────────────────
    _BARO_DRIFT_WARN = 5.0    # 該重新校準了
    _BARO_DRIFT_HIGH = 10.0   # 誤判離架已【無法撤銷】(火箭端 REVOKE_ALT_M)
    _BARO_DRIFT_CRIT = 20.0   # C 備援的 20m 地面保護【已失效】(DEPLOY_PEAK_MIN_M)

    def _track_baro_drift(self, ch: str, data):
        """地面待機時,RH 顯示的數字就是 ref_press 的漂移量。

        火箭沒動,rel_alt = 44330·(1 − (P/ref_press)^0.1903) 卻不是 0,
        就只可能是開機時記下的基準已經過期(天氣變化、在發射台等太久)。

        這不影響開傘判斷——cond_A 判的是「低於峰值 10m」,是**差值**,基準
        平移會抵消掉。但火箭端有兩道判**絕對值**的閘門會被打壞:

            漂 10 m(1.2 hPa) → 誤判離架後無法撤銷(rel_alt < REVOKE_ALT_M 永不成立)
            漂 20 m(2.4 hPa) → C 備援的 20m 地面保護失效(peak 一開始就超標)

        在發射台等幾小時就可能漂到。處置很簡單:按「校準 ALL」。
        只在 IDLE(stage 0)判定——飛起來之後 RH 本來就該是大數字。
        """
        if getattr(data, "stage", -1) != 0:
            self._prev_drift_level.pop(ch, None)
            return
        try:
            drift = abs(float(data.rel_height))
        except (TypeError, ValueError):
            return
        if drift >= self._BARO_DRIFT_CRIT:
            lvl = "crit"
        elif drift >= self._BARO_DRIFT_HIGH:
            lvl = "high"
        elif drift >= self._BARO_DRIFT_WARN:
            lvl = "warn"
        else:
            lvl = "ok"

        prev = self._prev_drift_level.get(ch)
        if lvl == prev:
            return                      # 同級不重複洗版
        self._prev_drift_level[ch] = lvl
        if lvl == "ok":
            if prev is not None:
                self.logger.info(f"✅ [{ch}] 氣壓基準已歸零({drift:.1f} m)")
            return
        hpa = drift / 8.3
        if lvl == "crit":
            self.logger.error(
                f"🔴 [{ch}] 氣壓基準漂移 {drift:.1f} m({hpa:.1f} hPa)"
                f"——C 備援的 20m 地面保護已失效!發射前務必按「校準 ALL」")
            self.broadcast_event(f"[🔴 {ch} 基準漂移 {drift:.0f}m]", "#FF3B30")
        elif lvl == "high":
            self.logger.error(
                f"🟠 [{ch}] 氣壓基準漂移 {drift:.1f} m({hpa:.1f} hPa)"
                f"——誤判離架後已無法撤銷。請按「校準 ALL」")
            self.broadcast_event(f"[🟠 {ch} 基準漂移 {drift:.0f}m]", "#FF9100")
        else:
            self.logger.warning(
                f"⚠ [{ch}] 氣壓基準漂移 {drift:.1f} m({hpa:.1f} hPa)"
                f"——建議按「校準 ALL」重設零點")


    # ── 上升段的 /dpl 二次確認 ────────────────────────────────────────────
    # 韌體只擋離架後前 10 秒，但頂點在 13.5~16.7 秒（3.0.8 的 81 組模擬）。
    # 也就是第 10 秒到頂點之間，文字指令 /dpl 會被火箭接受 —— 那時仍在上升、
    # 動壓最大，開傘等於解體。
    #
    # 但**不能無條件加確認**：真正的緊急情境（過了頂點、傘沒開）也是 stage 1，
    # 那時多一道手續是在偷走你最缺的東西。
    # 所以只擋「還在往上」這個唯一會出事的情況 —— 用遙測的 vz 判斷，
    # 下降中直接送出，行為與現在完全相同。
    _ASCENT_VZ = 2.0        # m/s，超過視為仍在上升

    def _ascent_guard(self, chans, action: str) -> bool:
        """回傳 True = 擋下來了（要求再輸入一次）。下降中或無資料時放行。

        用焦點頻道的 vz 判斷就夠 —— 兩塊板裝在**同一枚火箭**上，看到的是
        同一條軌跡。沒有資料時一律放行（不知道就不要擋緊急指令）。"""
        d = self.latest_data
        if d is None:
            self._pending_ascent = None
            return False
        if not (getattr(d, "stage", 0) == 1 and getattr(d, "vz", 0.0) > self._ASCENT_VZ):
            self._pending_ascent = None
            return False
        rising = [f"+{d.vz:.1f} m/s"]
        key = (action, tuple(chans))
        if getattr(self, "_pending_ascent", None) == key:
            self._pending_ascent = None
            self.logger.warning(f"⚠ 已確認：在上升段送出 {action.upper()}")
            return False
        self._pending_ascent = key
        self.logger.error(
            f"🛑 火箭仍在上升（{', '.join(rising)}）—— 上升段開傘會解體。"
            f"確定要送就再輸入一次同樣的指令。")
        self.broadcast_event("[🛑 上升中，再按一次確認]", "#FF3B30")
        return True

    def _batt_level(self, vf: float) -> str:
        """2S 鋰電電壓分級。刻意不換算成百分比——鋰電在變動負載下的
        電壓↔電量關係非線性,硬報一個百分比會給出比實際更精確的假象。"""
        if vf < 0 or vf < self._PYRO_LIVE_V:
            return "na"
        if vf < self._PYRO_CRIT_V:
            return "crit"
        if vf < self._PYRO_LOW_V:
            return "low"
        return "ok"

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

        # ═══════════════════════════════════════════════════════════
        # ★2026-08-01 電量分級告警【整段停用】—— 分壓電路沒有焊。
        #
        # 【不停用會怎樣】
        #   VF/VA 是 ADC 讀分壓後的電壓。分壓電路沒焊上去的話，那支腳是
        #   浮接的，ADC 讀到的是雜訊 —— 通常趴在 0 附近，偶爾被鄰腳耦合
        #   跳幾百 mV。換算出來的 VF 大約是 0.0~0.5V。
        #
        #   而判讀門檻是：
        #       VF < 6.6V  → 「電量危險，不應繼續飛行，立即更換電池」（紅色）
        #       VF < 1.0V  → 「保險絲熔斷」（_PYRO_LIVE_V，狀態列顯示 🔴熔斷）
        #
        #   兩條都會【永久成立】。後果不是「多幾行紅字」，而是：
        #     · 發射倒數時畫面上一直掛著紅色「電量危險」和「🔴熔斷」
        #     · 操作員在最需要相信畫面的時候，學會忽略紅色告警
        #     · 真的熔斷（誤觸發、電流走 safety shunt 燒斷保險絲 → 傘開不了）
        #       發生時，畫面看起來和現在一模一樣，沒有人會注意到
        #
        #   一個永遠為真的告警，比沒有告警更糟 —— 它會順便把其他告警一起
        #   訓練成雜訊。所以整段關掉，而不是留著讓它吼。
        #
        # 【焊上去之後怎麼恢復】
        #   把下面 if False: 改回 if lvl != "na":，並移除本註解區塊。
        #   門檻在 _PYRO_LOW_V / _PYRO_CRIT_V（7.0V / 6.6V，2S 鋰電）。
        #   狀態列的電量圖示（🔋/🪫）另外在 _refresh_channel_labels 停用。
        # ═══════════════════════════════════════════════════════════
        lvl = self._batt_level(vf)
        if False:   # ← 分壓電路焊好後改回 lvl != "na"
            prev_lvl = self._prev_batt_level.get(ch)
            if prev_lvl != lvl:
                self._prev_batt_level[ch] = lvl
                rank = {"ok": 0, "low": 1, "crit": 2}
                if prev_lvl is not None and rank[lvl] > rank[prev_lvl]:
                    if lvl == "crit":
                        self.logger.error(
                            f"🔋 [{ch}] 電量危險 {vf:.2f}V(2S 低於 {self._PYRO_CRIT_V}V,"
                            f"約剩 5%)——不應繼續飛行,立即更換電池")
                        self.broadcast_event(f"[🔋 {ch} 電量危險]", "#FF3B30")
                    else:
                        self.logger.warning(
                            f"🔋 [{ch}] 電量偏低 {vf:.2f}V(2S 低於 {self._PYRO_LOW_V}V,"
                            f"約剩 20%)——發射前建議更換")
                elif prev_lvl is not None and lvl == "ok":
                    self.logger.info(f"🔋 [{ch}] 電量恢復正常 {vf:.2f}V(已換電池?)")

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
        # (電量警告已移到本函式開頭的 early return 之前——擺這裡永遠走不到)

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