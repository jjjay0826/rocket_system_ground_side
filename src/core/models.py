from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple
import re
import math

# 數據模型
@dataclass
class SensorData:
    rotationRoll : float
    rotationPitch : float
    direction : float
    timestamp: datetime
    stage:int
    failedTasks:List[int]
    location: Optional[Tuple[float, float]]   # None = 沒有定位（不要編座標）
    
    # 擴展新版航電遙測屬性，全部設置預設值以防向後相容報錯
    timestamp_ms: int = 0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    pressure: float = 0.0
    rel_height: float = 0.0
    kfh_height: float = 0.0
    vz: float = 0.0
    total_accel: float = 0.0
    temp: float = 0.0
    raw_adc: int = 0
    flight_state: str = ""
    module_state: str = ""
    gnss_state: str = ""
    sv_visible: int = 0
    sv_used: int = 0
    buffer_val: int = 0
    count_val: int = 0
    cond_a_raw: int = 0
    cond_a_eff: int = 0
    cond_b_raw: int = 0
    cond_b_eff: int = 0
    peak_height: float = 0.0
    # pyro 電源監測(火箭端 PB1/PB0 ADC)。-1 = 該板無量測能力(舊韌體或 ADC 掛)
    v_fuse: float = -1.0   # 保險絲後端電壓:0V=熔斷(誤觸發已被 shunt 擋下)
    v_arm: float = -1.0    # arming 開關後端電壓:>0=已武裝(規範 4.6.7 遠端驗證)
    sd_writes: int = 0
    lora_seq: int = 0
    lora_success: int = 0
    lora_total: int = 0
    gs_timestamp: float = 0.0

    @classmethod
    def from_dict(cls, data: Dict[str, Any], timestamp: datetime = None) -> 'SensorData':
        """從 JSON 格式的字典創建 SensorData 物件"""
        try:
            # 轉換時間戳 (若傳入的是字串)
            ts = data.get('timestamp')
            if isinstance(ts, str):
                try:
                    ts = datetime.fromisoformat(ts)
                except ValueError:
                    ts = timestamp or datetime.now()
            else:
                ts = ts or timestamp or datetime.now()

            # 建立包含基本欄位字典
            # ★2026-08-01：None 要能原樣穿過 ZMQ（後端 → JSON null → GUI）。
            # 原本這裡對任何不合法的值都退回 (25.0, 121.5)，等於在
            # 反序列化時又把假座標種回去 —— 上游擋得再乾淨也沒用。
            location_val = data.get('location')
            if isinstance(location_val, (list, tuple)) and len(location_val) >= 2:
                location = (float(location_val[0]), float(location_val[1]))
            else:
                location = None

            kwargs = {
                "rotationRoll": float(data['rotationRoll']),
                "rotationPitch": float(data['rotationPitch']),
                "direction": float(data['direction']),
                "timestamp": ts,
                "stage": int(data.get('stage', 0)),
                "failedTasks": list(data.get('failedTasks', [])),
                "location": location
            }

            # 動態填寫其他可選欄位 (若存在於字典中)
            optional_fields = [
                "timestamp_ms", "ax", "ay", "az", "gx", "gy", "gz", "pressure",
                "rel_height", "kfh_height", "vz", "total_accel", "temp", "raw_adc",
                "flight_state", "module_state", "gnss_state", "sv_visible", "sv_used",
                "buffer_val", "count_val", "cond_a_raw", "cond_a_eff", "cond_b_raw",
                "cond_b_eff", "peak_height", "sd_writes", "lora_seq", "lora_success",
                "lora_total", "gs_timestamp", "v_fuse", "v_arm"
            ]
            for field in optional_fields:
                if field in data:
                    val = data[field]
                    if val is not None:
                        # 進行適當的型別轉換
                        if field in ["flight_state", "module_state", "gnss_state"]:
                            kwargs[field] = str(val)
                        elif field in ["timestamp_ms", "raw_adc", "sv_visible", "sv_used", 
                                      "buffer_val", "count_val", "cond_a_raw", "cond_a_eff", 
                                      "cond_b_raw", "cond_b_eff", "sd_writes", "lora_seq", 
                                      "lora_success", "lora_total"]:
                            kwargs[field] = int(val)
                        elif field == "gs_timestamp":
                            kwargs[field] = float(val)
                        else:
                            kwargs[field] = float(val)

            return cls(**kwargs)
        except KeyError as e:
            raise ValueError(f"Missing required field: {e}")
        except (ValueError, TypeError) as e:
            raise ValueError(f"Invalid data format: {e}")

    @classmethod
    def from_new_format(cls, line: str, timestamp: datetime = None) -> 'SensorData':
        """從新版航電板優化後的 ASCII 格式解析數據"""
        patterns = {
            'T': r'\bT(-?\d+)\b',
            'AX': r'\bAX([+-]?\d*\.?\d+)\b',
            'AY': r'\bAY([+-]?\d*\.?\d+)\b',
            'AZ': r'\bAZ([+-]?\d*\.?\d+)\b',
            'GX': r'\bGX([+-]?\d*\.?\d+)\b',
            'GY': r'\bGY([+-]?\d*\.?\d+)\b',
            'GZ': r'\bGZ([+-]?\d*\.?\d+)\b',
            'P': r'\bP([+-]?\d*\.?\d+)\b',
            'RH': r'\bRH([+-]?\d*\.?\d+)\b',
            'KH': r'\bKH([+-]?\d*\.?\d+)\b',
            'VZ': r'\bVZ([+-]?\d*\.?\d+)\b',
            'GA': r'\bGA([+-]?\d*\.?\d+)\b',
            'TC': r'\bTC([+-]?\d*\.?\d+)\b',
            'RAW': r'\bRAW(0x[0-9a-fA-F]+|\d+)\b',
            'ST': r'\bST:([A-Za-z0-9_-]+)\b',
            'MOD': r'\bMOD:?([0-9a-fA-F]+)\b',
            'GPS': r'\bGPS:([A-Za-z0-9_-]+(?:,\d+)?)\b',
            'SV': r'\bSV:(\d+),(\d+)\b',
            'BF': r'\bBF:(\d+),(\d+)\b',
            'CA': r'\bCA:(\d+),(\d+)\b',
            'CB': r'\bCB:(\d+),(\d+)\b',
            'C': r'\bC:([0-9a-fA-F]+)\b',
            'PK': r'\bPK([+-]?\d*\.?\d+)\b',
            'SD': r'\bSD(\d+)\b',
            'LR': r'\bLR:(\d+),(\d+),(\d+)\b',
            'SQ': r'\bSQ(\d+)\b',
            'VF': r'\bVF([+-]?\d*\.?\d+)\b',
            'VA': r'\bVA([+-]?\d*\.?\d+)\b',
            'LAT': r'\bLAT([+-]?\d*\.?\d+)\b',
            'LON': r'\bLON([+-]?\d*\.?\d+)\b',
        }
        
        extracted = {}
        for key, pattern in patterns.items():
            match = re.search(pattern, line)
            if match:
                if len(match.groups()) == 1:
                    extracted[key] = match.group(1)
                else:
                    extracted[key] = match.groups()
            else:
                extracted[key] = None

        if extracted['T'] is None:
            raise ValueError("Not in telemetry format: missing 'T' field")

        # ★截斷偵測:舊版只檢查 T,其餘欄位缺了就靜默填 0 —— RF 干擾切掉封包
        # 尾巴時,'T12345 AX+0.01' 會被當成一筆完整資料接受,產生高度 0、
        # 速度 0、**ST 0** 的假紀錄寫進 CSV。
        # 其中 ST 最危險:假的 stage=0 會重設地面站的 stage 邊緣偵測基準,
        # 下一筆真封包的 stage=2 就成了「0→2 的轉入」→ 在沒有新證據的情況下
        # 確認開傘指令(正是下行確認機制要防的假陽性)。
        # ST/MOD/GA 位於封包後段,任一缺席即代表尾巴被切掉 → 整筆丟棄。
        _required = [k for k in ('ST', 'MOD', 'GA') if extracted[k] is None]
        if _required:
            raise ValueError(
                f"Truncated telemetry frame: missing {','.join(_required)} "
                f"(len={len(line)})")

        t_val = int(extracted['T'])
        ax = float(extracted['AX']) if extracted['AX'] else 0.0
        ay = float(extracted['AY']) if extracted['AY'] else 0.0
        az = float(extracted['AZ']) if extracted['AZ'] else 0.0
        gx = float(extracted['GX']) if extracted['GX'] else 0.0
        gy = float(extracted['GY']) if extracted['GY'] else 0.0
        gz = float(extracted['GZ']) if extracted['GZ'] else 0.0
        p_val = float(extracted['P']) if extracted['P'] else 0.0
        rel_h = float(extracted['RH']) if extracted['RH'] else 0.0
        kfh_h = float(extracted['KH']) if extracted['KH'] else 0.0
        vz = float(extracted['VZ']) if extracted['VZ'] else 0.0
        g_val = float(extracted['GA']) if extracted['GA'] else 0.0
        tc = float(extracted['TC']) if extracted['TC'] else 0.0
        
        raw_adc = 0
        if extracted['RAW']:
            raw_str = extracted['RAW']
            raw_adc = int(raw_str, 16) if raw_str.lower().startswith('0x') else int(raw_str)
            
        flight_state_val = extracted['ST'] or ""
        flight_state = ""
        if flight_state_val.isdigit():
            stage = int(flight_state_val)
            # Map stage back to string representation
            reverse_stage_map = {
                0: "IDLE", 1: "ARMED", 2: "IGNITION", 3: "POWERED_FLIGHT",
                4: "BURNOUT", 5: "COASTING", 6: "APOGEE", 7: "PARACHUTE_DEPLOY",
                8: "DESCENT", 9: "TOUCHDOWN", 10: "AIRBAG_DEPLOY", 11: "LANDED"
            }
            flight_state = reverse_stage_map.get(stage, "IDLE")
        else:
            flight_state = flight_state_val
            stage_map = {
                "IDLE": 0, "ARMED": 1, "IGNITION": 2, "LAUNCH": 2, "LAUNCHED": 2,
                "POWERED_FLIGHT": 3, "BOOST": 3, "BURNOUT": 4, "COASTING": 5,
                "APOGEE": 6, "PARACHUTE_DEPLOY": 7, "PARACHUTE": 7, "DESCENT": 8,
                "TOUCHDOWN": 9, "AIRBAG_DEPLOY": 10, "AIRBAG": 10, "LANDED": 11
            }
            stage = stage_map.get(flight_state.upper(), 0)

        mod_str = extracted['MOD'] or ""
        failedTasks = []
        if len(mod_str) == 4 and all(c in '01' for c in mod_str):
            if mod_str[0] == '0': failedTasks.append(0)
            if mod_str[1] == '0': failedTasks.append(1)
            if mod_str[2] == '0': failedTasks.append(2)
            if mod_str[3] == '0': failedTasks.append(3)
        elif mod_str:
            try:
                val = int(mod_str, 16)
                if not (val & (1 << 3)): failedTasks.append(0)
                if not (val & (1 << 2)): failedTasks.append(1)
                if not (val & (1 << 1)): failedTasks.append(2)
                if not (val & (1 << 0)): failedTasks.append(3)
            except ValueError:
                pass

        gps_val = extracted['GPS'] or ""
        gnss_state = ""
        sv_visible, sv_used = 0, 0
        if "," in gps_val:
            parts = gps_val.split(",")
            status_code = parts[0]
            sv_visible = int(parts[1])
            sv_used = sv_visible
            gnss_state = "FIX_3D" if status_code == "1" else "NO_FIX"
        else:
            gnss_state = gps_val
            if extracted['SV']:
                sv_visible = int(extracted['SV'][0])
                sv_used = int(extracted['SV'][1])
            
        buffer_val, count_val = 0, 0
        if extracted['BF']:
            buffer_val = int(extracted['BF'][0])
            count_val = int(extracted['BF'][1])
            
        ca_raw, ca_eff, cb_raw, cb_eff = 0, 0, 0, 0
        if extracted.get('C'):
            try:
                val = int(extracted['C'], 16)
                ca_raw = 1 if (val & (1 << 3)) else 0
                ca_eff = 1 if (val & (1 << 2)) else 0
                cb_raw = 1 if (val & (1 << 1)) else 0
                cb_eff = 1 if (val & (1 << 0)) else 0
            except ValueError:
                pass
        else:
            if extracted['CA']:
                ca_raw = int(extracted['CA'][0])
                ca_eff = int(extracted['CA'][1])
            if extracted['CB']:
                cb_raw = int(extracted['CB'][0])
                cb_eff = int(extracted['CB'][1])
            
        pk_val = float(extracted['PK']) if extracted['PK'] else 0.0
        sd_val = int(extracted['SD']) if extracted['SD'] else 0
        
        lora_seq, lora_success, lora_total = 0, 0, 0
        if extracted['LR']:
            lora_seq = int(extracted['LR'][0])
            lora_success = int(extracted['LR'][1])
            lora_total = int(extracted['LR'][2])

        # 加速度估算 Roll/Pitch
        try:
            roll_rad = math.atan2(ay, az)
            pitch_rad = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            rotationRoll = roll_rad * 180.0 / math.pi
            rotationPitch = pitch_rad * 180.0 / math.pi
        except Exception:
            rotationRoll = 0.0
            rotationPitch = 0.0
            
        direction = 0.0

        # 發送序號:火箭每發一包遞增 1,地面端用跳號量化掉包率。
        # 只在 SQ 欄位存在時覆寫——舊格式的 LR:seq,ok,total(上面第一段)仍有效,
        # 沒有 SQ 就不能把它歸零。0 = 兩種格式都沒帶,不參與掉包計算。
        if extracted['SQ'] is not None:
            lora_seq = int(extracted['SQ'])

        # pyro 電源監測(缺欄位=舊韌體 → -1 表示「無此能力」,不可與 0V 熔斷混淆)
        v_fuse = float(extracted['VF']) if extracted['VF'] is not None else -1.0
        v_arm  = float(extracted['VA']) if extracted['VA'] is not None else -1.0

        # GPS 經緯度解析
        # ★GPS 宣稱已定位、座標卻缺席 = 封包在 LAT/LON 之前就被切掉。
        # 舊碼會靜默套用 (25.0, 121.5) —— 台北。那個座標會被當成「有效定位」
        # 畫在地圖上、寫進 CSV,落海搜救時是災難性的誤導。
        # 這種情況降級為 NO_FIX,讓地面站照「沒有定位」處理。
        if gnss_state == "FIX_3D" and (extracted['LAT'] is None or extracted['LON'] is None):
            gnss_state = "NO_FIX"
        # ★2026-08-01：沒有定位就是 None，不要編一個座標出來。
        # 舊碼填 (25.0, 121.5)＝台北。上面那段註解已經寫了它有多危險，
        # 但只擋掉了「畫在地圖主標記上」這一條路；事件標記、CSV、
        # 以及任何未來的消費端都照用那個假座標。實際發生過的事：
        # 火箭在旭海的桌上、GPS 還沒定位，地圖上卻出現一個位在台北的
        # [CMD] Reset Angle 標記。
        # None 會讓所有 `if location:` 自然擋下，而且新寫的程式碼不可能
        # 「不小心」把 None 當成合法座標用。
        if extracted['LAT'] is None or extracted['LON'] is None:
            location = None
        else:
            location = (float(extracted['LAT']), float(extracted['LON']))

        return cls(
            rotationRoll=rotationRoll,
            rotationPitch=rotationPitch,
            direction=direction,
            timestamp=timestamp or datetime.now(),
            stage=stage,
            failedTasks=failedTasks,
            location=location,
            timestamp_ms=t_val,
            ax=ax, ay=ay, az=az,
            gx=gx, gy=gy, gz=gz,
            pressure=p_val,
            rel_height=rel_h,
            kfh_height=kfh_h,
            vz=vz,
            total_accel=g_val,
            temp=tc,
            raw_adc=raw_adc,
            flight_state=flight_state,
            module_state=mod_str,
            gnss_state=gnss_state,
            sv_visible=sv_visible,
            sv_used=sv_used,
            buffer_val=buffer_val,
            count_val=count_val,
            cond_a_raw=ca_raw,
            cond_a_eff=ca_eff,
            cond_b_raw=cb_raw,
            cond_b_eff=cb_eff,
            peak_height=pk_val,
            v_fuse=v_fuse,
            v_arm=v_arm,
            sd_writes=sd_val,
            lora_seq=lora_seq,
            lora_success=lora_success,
            lora_total=lora_total
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "rotationRoll": self.rotationRoll,
            "rotationPitch": self.rotationPitch,
            "direction": self.direction,
            "timestamp": self.timestamp,
            "stage": self.stage,
            "failedTasks": self.failedTasks,
            "location": self.location,
            "timestamp_ms": self.timestamp_ms,
            "ax": self.ax,
            "ay": self.ay,
            "az": self.az,
            "gx": self.gx,
            "gy": self.gy,
            "gz": self.gz,
            "pressure": self.pressure,
            "rel_height": self.rel_height,
            "kfh_height": self.kfh_height,
            "vz": self.vz,
            "total_accel": self.total_accel,
            "temp": self.temp,
            "raw_adc": self.raw_adc,
            "flight_state": self.flight_state,
            "module_state": self.module_state,
            "gnss_state": self.gnss_state,
            "sv_visible": self.sv_visible,
            "sv_used": self.sv_used,
            "buffer_val": self.buffer_val,
            "count_val": self.count_val,
            "cond_a_raw": self.cond_a_raw,
            "cond_a_eff": self.cond_a_eff,
            "cond_b_raw": self.cond_b_raw,
            "cond_b_eff": self.cond_b_eff,
            "peak_height": self.peak_height,
            "sd_writes": self.sd_writes,
            "lora_seq": self.lora_seq,
            "lora_success": self.lora_success,
            "lora_total": self.lora_total,
            "v_fuse": self.v_fuse,
            "v_arm": self.v_arm
        }
@dataclass
class LogData:
    content : str
    timestamp: datetime

    @classmethod
    def new(cls, content:str, timestamp: datetime = None) -> 'LogData':
        """依照 content 創建 LogData 物件"""
        try:
            return cls(
                content=content,
                timestamp=timestamp or datetime.now()
            )
        except KeyError as e:
            raise ValueError(f"Missing required field: {e}")
        except ValueError as e:
            raise ValueError(f"Invalid data format: {e}")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "content": self.content,
            "timestamp": self.timestamp,
        }