"""飛行階段顯示。

★ 權威來源是火箭端的 `FlightState_t`（5 個狀態），不是這裡的清單。
  遙測 `ST:` 欄送的就是那個列舉的整數值，本檔直接拿它當索引。

  2026-07-30 之前這裡放的是一份 12 狀態的清單
  （IDLE/ARMED/IGNITION/POWERED_FLIGHT/BURNOUT/COASTING/APOGEE/
    PARACHUTE_DEPLOY/DESCENT/TOUCHDOWN/AIRBAG_DEPLOY/LANDED），
  **那套方案從未在火箭端實作**。後果不只是名字錯：

    火箭送 ST:2（正在放傘）→ 螢幕顯示「IGNITION」、圖表打上綠色 IGNITION 標記
    火箭送 ST:4（已落地）  → 螢幕顯示「BURNOUT」，後面 7 格永遠灰著

  而且 T0 被錨定在 index 2，所以所有「T+x.xxs」都是**從開傘那一刻起算**的。

  火箭端無法產生那 12 態中的幾個 —— 它看不到點火（最早知道的是離架後
  2.5g×200ms，已是點火後約 1.3 秒），也沒有獨立的頂點判定。所以方向是
  「地面端吻合火箭端」，缺的資訊由地面站從既有遙測自行推導（見 mark_derived）。
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


# ── 火箭端 FlightState_t（firmware-rocket/Core/Src/main.c:292）──
# 改這裡之前先確認韌體的列舉真的改了。tests/test_crossrepo_protocol.py 會比對。
ROCKET_STAGES = ["IDLE", "LAUNCHED", "DEPLOYING", "DEPLOYED", "LANDED"]

# 中文註解，直接顯示在名稱後面，避免「DEPLOYING 到底是要開還是開完了」的誤讀
STAGE_HINT = {
    0: "地面靜置",
    1: "已離架",
    2: "開傘訊號輸出中",
    3: "傘已開・下降",
    4: "已落地",
}

# 進入這些狀態時視為「事件」，回傳給 main_window 去圖表與地圖打標
EVENT_STAGES = {
    1: ("LAUNCH", "#00E676"),
    2: ("PARACHUTE_DEPLOY", "#D500F9"),
    3: ("DESCENT", "#FFD600"),
    4: ("LANDED", "#00E5FF"),
}

# T0 = 離架（火箭能量到的最早時刻）。不是開傘，也不是點火 —— 火箭看不到點火。
T0_STAGE = 1


class StageDisplayer:
    def __init__(self, list_widget: QListWidget):
        self.list_widget: QListWidget = list_widget
        self.current_stage = -1
        self.visited_stages = set()
        self.marked_events = set()
        self.stages = list(ROCKET_STAGES)
        self.stage_times = {}      # stage -> 首次達到的時間戳
        self.derived = []          # [(name, timestamp, color)] 地面站推導出來的事件

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
    def _labels(self):
        out = []
        for i, name in enumerate(self.stages):
            hint = STAGE_HINT.get(i, "")
            txt = f"  {name}" + (f"　{hint}" if hint else "")
            if i in self.stage_times:
                if i == T0_STAGE:
                    txt += "  (T0)"
                elif T0_STAGE in self.stage_times:
                    dt = (self.stage_times[i] - self.stage_times[T0_STAGE]).total_seconds()
                    txt += f"  (T+{dt:.2f}s)"
            out.append(txt)
        # 推導事件排在真實狀態之後，用「·」開頭標示它不是火箭送來的
        for name, ts, _ in self.derived:
            txt = f"  · {name}"
            if T0_STAGE in self.stage_times:
                dt = (ts - self.stage_times[T0_STAGE]).total_seconds()
                txt += f"  (T+{dt:.2f}s)"
            out.append(txt + "　地面推導")
        return out

    def _rebuild(self):
        self.list_widget.clear()
        self.list_widget.addItems(self._labels())
        self._repaint()

    def _repaint(self):
        n = len(self.stages)
        for i in range(self.list_widget.count()):
            item = self.list_widget.item(i)
            if not item:
                continue
            if i >= n:                                   # 推導事件：淡藍
                item.setForeground(QBrush(QColor(0, 0, 0)))
                item.setBackground(QBrush(QColor(190, 215, 235)))
            elif i < self.current_stage:
                item.setForeground(QBrush(QColor(0, 0, 0)))
                # 沒造訪過卻已經越過 = 該狀態的遙測整段掉包
                item.setBackground(QBrush(QColor(150, 200, 150) if i in self.visited_stages
                                          else QColor(180, 70, 70)))
            elif i == self.current_stage:
                item.setForeground(QBrush(QColor(0, 0, 0)))
                item.setBackground(QBrush(QColor(200, 200, 200)))
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

        # ★超出範圍就丟掉。舊碼會 self.stages[stage] 直接 IndexError；
        #   而且若韌體真的改成 12 態，這裡會安靜地擋住而不是畫出錯誤的名字。
        if stage < 0 or stage >= len(self.stages):
            if stage >= len(self.stages):
                logger.error(f"[STAGE] 收到 ST:{stage}，超出火箭端已知的 "
                             f"{len(self.stages)} 個狀態 —— 韌體與地面站版本不符?")
            return is_new_event, event_name, event_color

        if timestamp is None:
            timestamp = datetime.now()
        if stage not in self.stage_times:
            self.stage_times[stage] = timestamp
        # 容錯：直接收到 >T0 的狀態卻沒有 T0（開機時已在飛，或前段遙測全掉）
        if stage > T0_STAGE and T0_STAGE not in self.stage_times:
            self.stage_times[T0_STAGE] = timestamp
            logger.warning("[STAGE] 沒收到離架幀，T0 以第一筆飛行幀代替 —— 時間軸不準")

        if stage != self.current_stage:
            old = self.stages[self.current_stage] if self.current_stage >= 0 else "NONE"
            suffix = ""
            if stage == T0_STAGE:
                suffix = " (T0)"
            elif stage > T0_STAGE and T0_STAGE in self.stage_times:
                dt = (self.stage_times[stage] - self.stage_times[T0_STAGE]).total_seconds()
                suffix = f" (T+{dt:.2f}s)"
            logger.info(f"[STAGE] {old} -> {self.stages[stage]}{suffix}")

            for skipped in range(max(0, self.current_stage + 1), stage):
                if skipped not in self.visited_stages:
                    logger.warning(f"[STAGE SKIPPED] 「{self.stages[skipped]}」"
                                   f"從未收到 —— 該段遙測掉包")

        self.visited_stages.add(stage)
        self.current_stage = stage
        self._rebuild()
        return is_new_event, event_name, event_color

    # ──────────────────────────────────────────────────────────────
    def mark_derived(self, name: str, timestamp: datetime = None,
                     color: str = "#FF9100"):
        """地面站自己從遙測推導出來的事件（火箭端沒有對應的狀態）。

        火箭的 5 個狀態刻意保持精簡 —— 它們是開傘決策實際在用的東西，
        多加一個狀態就要重新稽核所有 `flight_state == ...` 的比較。
        但遙測裡本來就帶著足以推導其他事件的資料：

            BURNOUT  ← GA(total_accel) 跌破 1.15
            APOGEE   ← 高度峰值的時刻
            AIRBAG   ← MSG INFO Airbag inflation started

        這些標成「地面推導」，與火箭送來的狀態視覺上分開，
        免得日後有人把推導值當成火箭的實測回報。

        重複同名只記第一次。回傳是否為新事件。
        """
        if any(n == name for n, _, _ in self.derived):
            return False
        if timestamp is None:
            timestamp = datetime.now()
        self.derived.append((name, timestamp, color))
        rel = ""
        if T0_STAGE in self.stage_times:
            rel = f" (T+{(timestamp - self.stage_times[T0_STAGE]).total_seconds():.2f}s)"
        logger.info(f"[DERIVED] {name}{rel}　（地面站推導，非火箭回報）")
        self._rebuild()
        return True

    # ──────────────────────────────────────────────────────────────
    def reset(self):
        """重置火箭任務階段顯示 UI 狀態與內部歷史紀錄"""
        self.current_stage = -1
        self.visited_stages.clear()
        self.marked_events.clear()
        self.stage_times.clear()
        self.derived.clear()
        self._rebuild()
