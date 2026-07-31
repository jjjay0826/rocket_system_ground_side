"""飛行階段顯示。

★ 權威來源是火箭端的 `FlightState_t`（5 個狀態），不是這裡的清單。
  遙測 `ST:` 欄送的就是那個列舉的整數值。

  ── 兩次改動的來龍去脈 ──────────────────────────────────────────────

  2026-07-30 之前這裡放的是一份 12 狀態清單
  （IDLE/ARMED/IGNITION/POWERED_FLIGHT/BURNOUT/COASTING/APOGEE/
    PARACHUTE_DEPLOY/DESCENT/TOUCHDOWN/AIRBAG_DEPLOY/LANDED），
  而且【直接拿 ST 當索引】。那套方案從未在火箭端實作，後果不只是名字錯：

    火箭送 ST:2（正在放傘）→ 螢幕顯示「IGNITION」、圖表打上綠色 IGNITION 標記
    火箭送 ST:4（已落地）  → 螢幕顯示「BURNOUT」，後面 7 格永遠灰著

  而且 T0 被錨定在 index 2，所以所有「T+x.xxs」都是從開傘那一刻起算的。

  2026-07-30 改成只列火箭真有的 5 個狀態。正確，但畫面只剩 5 行 ——
  看不出「現在飛到哪裡了」，燃盡、頂點這些關鍵時刻沒有位置可以標。

  2026-08-01（本版）：把完整的飛行序列列回來，但【不再用 ST 當索引】。
  火箭的 5 個狀態各自對應到序列裡的一個特定列（ST_ROW），其餘的列由
  地面站從遙測推導後點亮（mark_derived）。兩者在畫面上用不同顏色分開，
  免得有人把推導值當成火箭的實測回報。

  火箭端無法產生序列中的幾個 —— 它看不到點火（最早知道的是離架後
  2.5g×200ms，已是點火後約 1.3 秒），也沒有獨立的頂點判定。所以方向仍是
  「地面端吻合火箭端」，缺的資訊由地面站自己補。

  ⚠ AIRBAG_DEPLOY 已於 2026-07-31 移除（氣囊取消，PA0 併入降落傘迴路），
    所以序列是 11 列而不是原本的 12 列。
"""
from PyQt6.QtGui import QColor, QBrush
from PyQt6.QtWidgets import QListWidget, QStyledItemDelegate, QAbstractItemView, QStyle
from PyQt6.QtCore import Qt
from datetime import datetime
import logging

logger = logging.getLogger("src.gui.stage_display")


class CustomDelegate(QStyledItemDelegate):
    def sizeHint(self, option, index):
        size = super().sizeHint(option, index)
        size.setHeight(30)
        self.padding = 10
        return size

    def paint(self, painter, option, index):
        # 💡 移除滑鼠懸停 (hover)、選取與焦點狀態，防止不可互動元件顯示高亮白底干擾閱讀
        option.state &= ~QStyle.StateFlag.State_MouseOver
        option.state &= ~QStyle.StateFlag.State_Selected
        option.state &= ~QStyle.StateFlag.State_HasFocus
        super().paint(painter, option, index)


# ── 完整飛行序列。(名稱, 中文提示, 來源) ───────────────────────────────
#    "rkt" = 火箭 ST 欄直接回報   "gnd" = 地面站從遙測推導
#
# ★ "rkt" 那五列的名稱【必須與韌體的 FlightState_t 逐字相同】。
#   畫面上寫 PARACHUTE_DEPLOY、原始碼裡叫 DEPLOYING，事後要對照的人
#   會 grep 不到。中文提示負責可讀性，英文名負責可追溯。
#   tests/test_stage_names.py 會比對韌體的列舉。
FLIGHT_SEQUENCE = [
    ("IDLE",       "地面靜置",       "rkt"),   # ST:0
    ("ARMED",      "已解鎖",         "gnd"),   # ← MSG "MANUAL ARMED"
    ("IGNITION",   "點火",           "gnd"),   # ← 火箭看不到，由離架回推
    ("LAUNCHED",   "離架・推力段",    "rkt"),   # ST:1
    ("BURNOUT",    "燃盡",           "gnd"),   # ← GA 跌破 1.15
    ("COASTING",   "慣性上升",       "gnd"),
    ("APOGEE",     "頂點",           "gnd"),   # ← 高度峰值 / 進入 ST:2 時
    ("DEPLOYING",  "開傘訊號輸出中",  "rkt"),   # ST:2
    ("DEPLOYED",   "傘已開・下降",    "rkt"),   # ST:3
    ("TOUCHDOWN",  "觸地／觸水",      "gnd"),
    ("LANDED",     "已落地",         "rkt"),   # ST:4
]

# 火箭 ST 值 → 序列列號。★改韌體的 FlightState_t 之前先回來改這裡。
#   tests/test_crossrepo_protocol.py 會比對韌體的列舉。
ST_ROW = {0: 0, 1: 3, 2: 7, 3: 8, 4: 10}
ROW_ST = {v: k for k, v in ST_ROW.items()}

# 進入這些 ST 時視為「事件」，回傳給 main_window 去圖表與地圖打標
EVENT_STAGES = {
    1: ("LAUNCH", "#00E676"),
    2: ("PARACHUTE_DEPLOY", "#D500F9"),
    3: ("DESCENT", "#FFD600"),
    4: ("LANDED", "#00E5FF"),
}

# T0 = 離架（火箭能量到的最早時刻）。不是開傘，也不是點火 —— 火箭看不到點火。
T0_ROW = ST_ROW[1]

# 相容別名：外部若還有人 import 這兩個名字
ROCKET_STAGES = [n for n, _, src in FLIGHT_SEQUENCE if src == "rkt"]
T0_STAGE = T0_ROW


class StageDisplayer:
    def __init__(self, list_widget: QListWidget):
        self.list_widget: QListWidget = list_widget
        self.current_row = -1
        self.visited_rows = set()          # 火箭真的回報過的列
        self.derived_rows = {}             # row -> (顯示名, timestamp)
        self.marked_events = set()
        self.row_times = {}                # row -> 首次達到的時間戳
        self.extra = []                    # 不在序列裡的推導事件，附在最後

        # 暫時移除滑鼠點擊/選中高亮變白功能：禁用選取與焦點
        self.list_widget.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.list_widget.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.list_widget.setStyleSheet("""
            QListWidget::item:hover {
                background-color: transparent;
            }
        """)
        self._rebuild()
        self.list_widget.setItemDelegate(CustomDelegate())

    # ──────────────────────────────────────────────────────────────
    def _rel(self, ts):
        if ts is None or T0_ROW not in self.row_times:
            return ""
        return f"  (T+{(ts - self.row_times[T0_ROW]).total_seconds():.2f}s)"

    def _labels(self):
        out = []
        for i, (name, hint, src) in enumerate(FLIGHT_SEQUENCE):
            shown = self.derived_rows.get(i, (name, None))[0] if src == "gnd" else name
            txt = f"  {shown}" + (f"　{hint}" if hint else "")
            ts = self.row_times.get(i)
            if ts is not None:
                txt += "  (T0)" if i == T0_ROW else self._rel(ts)
            if src == "gnd":
                txt += "　推導" if i in self.derived_rows else "　—"
            out.append(txt)
        for name, ts, _ in self.extra:
            out.append(f"  · {name}{self._rel(ts)}　地面推導")
        return out

    def _rebuild(self):
        self.list_widget.clear()
        self.list_widget.addItems(self._labels())
        self._repaint()

    def _repaint(self):
        n = len(FLIGHT_SEQUENCE)
        for i in range(self.list_widget.count()):
            item = self.list_widget.item(i)
            if not item:
                continue
            item.setForeground(QBrush(QColor(0, 0, 0)))
            if i >= n:                                    # 序列外的推導事件
                item.setBackground(QBrush(QColor(190, 215, 235)))
                continue
            src = FLIGHT_SEQUENCE[i][2]
            if i == self.current_row:                     # 目前所在
                item.setBackground(QBrush(QColor(200, 200, 200)))
            elif src == "gnd":
                # 推導列：點亮＝淡藍，沒點亮＝灰（不算「掉包」，火箭本來就不送）
                if i in self.derived_rows:
                    item.setBackground(QBrush(QColor(190, 215, 235)))
                else:
                    item.setBackground(QBrush(QColor(254, 254, 254)))
                    item.setForeground(QBrush(QColor(160, 160, 160)))
            elif i < self.current_row:
                # 火箭列且已越過：沒造訪過 = 該狀態的遙測整段掉包
                item.setBackground(QBrush(QColor(150, 200, 150) if i in self.visited_rows
                                          else QColor(180, 70, 70)))
            else:
                item.setBackground(QBrush(QColor(254, 254, 254)))
                item.setForeground(QBrush(QColor(140, 140, 140)))

    # ──────────────────────────────────────────────────────────────
    def update(self, stage: int, timestamp: datetime = None):
        """stage = 遙測 ST: 欄（火箭端 FlightState_t，0~4）。

        首次進入某個事件狀態時回傳 (True, 事件名, 顏色)，其餘回 (False, None, None)。
        """
        is_new_event, event_name, event_color = False, None, None

        if stage in EVENT_STAGES and stage not in self.marked_events:
            self.marked_events.add(stage)
            is_new_event = True
            event_name, event_color = EVENT_STAGES[stage]

        # ★不再拿 ST 當列索引 —— 走對應表。表裡沒有就丟掉並大聲說。
        row = ST_ROW.get(stage)
        if row is None:
            logger.error(f"[STAGE] 收到 ST:{stage}，不在火箭端已知的 "
                         f"{sorted(ST_ROW)} 之內 —— 韌體與地面站版本不符?")
            return is_new_event, event_name, event_color

        if timestamp is None:
            timestamp = datetime.now()
        if row not in self.row_times:
            self.row_times[row] = timestamp
        # 容錯：直接收到 >T0 的狀態卻沒有 T0（開機時已在飛，或前段遙測全掉）
        if row > T0_ROW and T0_ROW not in self.row_times:
            self.row_times[T0_ROW] = timestamp
            logger.warning("[STAGE] 沒收到離架幀，T0 以第一筆飛行幀代替 —— 時間軸不準")

        if row != self.current_row:
            old = (FLIGHT_SEQUENCE[self.current_row][0]
                   if self.current_row >= 0 else "NONE")
            suffix = " (T0)" if row == T0_ROW else self._rel(self.row_times.get(row))
            logger.info(f"[STAGE] {old} -> {FLIGHT_SEQUENCE[row][0]}{suffix}")

            # 只有【火箭列】被跳過才算遙測掉包；推導列本來就不是火箭送的
            for skipped in range(max(0, self.current_row + 1), row):
                if (FLIGHT_SEQUENCE[skipped][2] == "rkt"
                        and skipped not in self.visited_rows):
                    logger.warning(f"[STAGE SKIPPED] 「{FLIGHT_SEQUENCE[skipped][0]}」"
                                   f"從未收到 —— 該段遙測掉包")

        self.visited_rows.add(row)
        self.current_row = row
        self._rebuild()
        return is_new_event, event_name, event_color

    # ──────────────────────────────────────────────────────────────
    def mark_derived(self, name: str, timestamp: datetime = None,
                     color: str = "#FF9100"):
        """地面站自己從遙測推導出來的事件（火箭端沒有對應的狀態）。

        火箭的 5 個狀態刻意保持精簡 —— 它們是開傘決策實際在用的東西，
        多加一個狀態就要重新稽核所有 `flight_state == ...` 的比較。
        但遙測裡本來就帶著足以推導其他事件的資料：

            ARMED     ← MSG "MANUAL ARMED"
            BURNOUT   ← GA(total_accel) 跌破 1.15
            APOGEE    ← 高度峰值的時刻 / 進入 ST:2

        名稱若對得上序列裡的某一列（前綴比對，所以 "APOGEE 1040m" 也算
        APOGEE），就點亮那一列並把完整名稱顯示上去；對不上的附在最後面。

        重複同名只記第一次。回傳是否為新事件。
        """
        if timestamp is None:
            timestamp = datetime.now()
        key = name.split()[0].upper() if name else ""
        for i, (row_name, _, src) in enumerate(FLIGHT_SEQUENCE):
            if src == "gnd" and row_name == key:
                if i in self.derived_rows:
                    return False
                self.derived_rows[i] = (name, timestamp)
                self.row_times.setdefault(i, timestamp)
                logger.info(f"[DERIVED] {name}{self._rel(timestamp)}"
                            f"　（地面站推導，非火箭回報）")
                self._rebuild()
                return True
        # 不在序列裡：附在最後
        if any(n == name for n, _, _ in self.extra):
            return False
        self.extra.append((name, timestamp, color))
        logger.info(f"[DERIVED] {name}{self._rel(timestamp)}　（地面站推導，非火箭回報）")
        self._rebuild()
        return True

    # ──────────────────────────────────────────────────────────────
    def reset(self):
        """重置火箭任務階段顯示 UI 狀態與內部歷史紀錄"""
        self.current_row = -1
        self.visited_rows.clear()
        self.derived_rows.clear()
        self.marked_events.clear()
        self.row_times.clear()
        self.extra.clear()
        self._rebuild()
