import logging
import time
from typing import Tuple, Any

class LoraCommand:
    """LoRa 遠端控制指令金鑰定義。

    ⚠ 韌體端用 strcmp() 做【逐字比對】（main.c:795/815/841/917/918）。
      這裡差一個字元，指令就靜默失效 —— 火箭只會回一句
      "Unknown CMD - check exact secret string"，而那句在忙亂的發射台上
      很容易被當成雜訊。tests/test_crossrepo_protocol.py 會逐字比對，
      改動任何一條都會讓那支測試變紅。
    """
    ARM = ("arm", b"#CMD:ARM_SYSTEM_SALT7763#\r\n", "系統遠端解鎖 (ARM)")
    DPL = ("dpl", b"#CMD:FORCE_DPL_SALT9981#\r\n", "遠端強制開傘 (DPL)")
    CAL = ("cal", b"#CMD:RECAL_SALT5566#\r\n", "氣壓零點重校 (RECAL)")
    # 地面測試模式：解除 PB6 手動發火的閘門，10 分鐘後自動失效。
    # 規範 4.5.3 要求「回收系統應在模擬觸發條件下地面測試」，而 PB6 需要
    # IDLE + ARM + GNDTEST 三者齊備。以前只能靠 USB 手打這一串——插著線
    # 做發火測試很不方便，而且沒人記得住 24 個字元。
    GND = ("gndtest", b"#CMD:GNDTEST_SALT3310#\r\n", "地面測試模式 (GNDTEST, 10min)")
    # 韌體認得關閉指令，但地面站以前【送不出來】—— 開得了、關不掉，只能等
    # 10 分鐘逾時或插 USB。發射倒數時不會有人想去插 USB。
    # 這個缺口是 2026-08-01 加上「韌體認得的指令地面站都送得出來」這條
    # 反向斷言之後才浮出來的：兩邊各寫一份，缺了一條不會有任何東西變紅。
    GND_OFF = ("gndtest_off", b"#CMD:GNDTEST_OFF#\r\n", "關閉地面測試模式")
    # ★2026-07-31 氣囊已移除，PA0 併入降落傘發火迴路。韌體對這條會明確拒收。
    #   保留定義只是為了讓舊版地面站送來的 abg 在協定層仍有名字可對；
    #   【不要】把任何按鈕或指令接回這裡。
    ABG = ("abg", b"#CMD:OPEN_ABG_SALT8872#\r\n", "開啟氣囊 (ABG) — 已停用")

    @classmethod
    def get_token(cls, action: str) -> Tuple[bytes, str]:
        """依據 action 取得對應防偽秘鑰 Token 與人類可讀標籤"""
        action_lower = action.lower()
        if action_lower == "arm":
            return cls.ARM[1], cls.ARM[2]
        elif action_lower == "dpl":
            return cls.DPL[1], cls.DPL[2]
        elif action_lower == "abg":
            return cls.ABG[1], cls.ABG[2]
        elif action_lower == "cal":
            return cls.CAL[1], cls.CAL[2]
        elif action_lower == "gndtest":
            return cls.GND[1], cls.GND[2]
        elif action_lower == "gndtest_off":
            return cls.GND_OFF[1], cls.GND_OFF[2]
        elif action_lower.startswith("setch_"):
            # 換頻:#CMD:SETCH_72# → 922.125MHz(E22-900T22D: 850.125 + ch)
            ch = action_lower.split("_", 1)[1]
            return f"#CMD:SETCH_{ch}#\r\n".encode("utf-8"), f"切換 LoRa 頻道 (CH{ch})"
        else:
            token = f"#CMD:{action}#\r\n".encode('utf-8')
            label = f"自訂遠端指令 ({action})"
            return token, label


class LoraProtocolHandler:
    """LoRa 通訊協定處理器（支援獨立頻道 ch1/ch2 實例化）"""
    def __init__(self, channel_id: str = "ch1"):
        self.channel_id = channel_id
        self.logger = logging.getLogger(f"LoraProtocol_{channel_id.upper()}")

    def send_command(
        self,
        communicator: Any,
        action: str,
        repeat_count: int = 4,
        burst_interval: float = 0.7
    ) -> Tuple[bool, int, str]:
        """
        透過傳輸介面 (Communicator) 重複連發下傳遠端控制指令
        :param communicator: 具備 send_bytes 方法的傳輸物件 (例如 SerialCommunicator)
        :param action: 指令代碼 (如 arm, dpl, abg)
        :param repeat_count: Burst 發送次數 (預設 4 次)
        :param burst_interval: 發送間隔時間以秒為單位 (預設 0.7s / 700ms，避開火箭 2Hz 時間窗口)
        :return: (是否成功傳送至少 1 幀, 成功傳送之幀數, 結果說明訊息)
        """
        raw_token, cmd_label = LoraCommand.get_token(action)
        self.logger.info(
            f"🟦 [CMD] Transmitting /{action} ({cmd_label}) over LoRa ({repeat_count}x bursts, {int(burst_interval * 1000)}ms interval)..."
        )

        # ★2026-08-01：hasattr 提到迴圈外（迴圈內不會變），而且
        # 【最後一發之後不再 sleep】。
        #
        # 原本 4 發各睡 0.7s = 2.8s，但最後那一次睡完什麼事都沒有 ——
        # 純粹是讓後端晚 0.7 秒才回覆 GUI。緊急開傘的時候那 0.7 秒是
        # 操作員盯著畫面等確認的 0.7 秒（傘下 6~7m/s ≈ 4~5 公尺高度）。
        # 第一發在毫秒內就出去了，所以指令本身沒有變快，變快的是
        # 「知道它出去了」。
        # 序列埠離線時更明顯：4 次全失敗仍要空等 2.8s 才報錯，現在 2.1s。
        can_send = bool(communicator) and hasattr(communicator, 'send_bytes')
        if not can_send:
            msg = (f"🟥 [CMD] Transmit failed for /{action} ({cmd_label}): "
                   f"no serial transport on {self.channel_id.upper()}.")
            self.logger.error(msg)
            return False, 0, msg

        sent_success = 0
        for i in range(repeat_count):
            ok = communicator.send_bytes(raw_token)
            if ok:
                sent_success += 1
            # 逐發記錄：事後要回答「指令到底有沒有送出去、什麼時候」時，
            # 只有總結那一行是不夠的。
            self.logger.info(
                f"🟦 [CMD] /{action} burst {i + 1}/{repeat_count} "
                f"{'sent' if ok else 'FAILED'}")
            if i + 1 < repeat_count:
                time.sleep(burst_interval)

        if sent_success > 0:
            msg = f"🟦 [CMD] Successfully transmitted /{action} ({cmd_label}) over {self.channel_id.upper()} ({sent_success}/{repeat_count} bursts)."
            self.logger.info(msg)
            return True, sent_success, msg
        else:
            msg = f"🟥 [CMD] Transmit failed for /{action} ({cmd_label}): Serial port offline."
            self.logger.error(msg)
            return False, 0, msg
