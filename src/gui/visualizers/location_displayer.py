import logging
from typing import Tuple

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWebEngineCore import QWebEnginePage

class NonNavigablePage(QWebEnginePage):
    """自訂 QWebEnginePage，防止使用者因誤點地圖超連結（如版權資訊）跳轉至外部網頁"""
    def acceptNavigationRequest(self, url, navigation_type, is_main_frame):
        if navigation_type == QWebEnginePage.NavigationType.NavigationTypeLinkClicked:
            return False
        return super().acceptNavigationRequest(url, navigation_type, is_main_frame)

class LocationDisplayer:
    def __init__(self, widget: QWidget, initial_location: Tuple[float, float] = (23.5, 121.5)):
        """
        初始化LocationDisplayer
        
        Args:
            widget (QWidget): 用於顯示地圖的Qt widget
            initial_location (Tuple[float, float]): 初始位置的(緯度, 經度)，默認為台灣中心位置
        """
        self.widget = widget
        self.layout = QVBoxLayout(widget)
        self.layout.setSpacing(0)
        self.layout.setContentsMargins(0, 0, 0, 0) 

        self.web_view = QWebEngineView()
        self.web_view.setPage(NonNavigablePage(self.web_view))
        self.layout.addWidget(self.web_view)
        
        self.logger = logging.getLogger(__name__)

        self.current_location = initial_location
        self.map_initialized = False
        self.create_map(initial_location)

    def create_map(self, location: Tuple[float, float]):
        """創建新的地圖並載入"""
        lat, lng = location
        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8" />
            <meta name="viewport" content="width=device-width, initial-scale=1.0">
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
            <style>
                html, body, #map {{ width: 100%; height: 100%; margin: 0; padding: 0; }}
                /* 停用版權與標示區域的點擊事件，防止誤觸跳轉 */
                .leaflet-control-attribution, .leaflet-control-attribution a {{
                    pointer-events: none !important;
                    cursor: default !important;
                    text-decoration: none !important;
                }}
                /* ★座標疊層：不依賴 Leaflet，地圖掛掉時它就是唯一的資訊 */
                #coord {{
                    position: fixed; left: 0; top: 0; z-index: 9999;
                    background: rgba(0,0,0,.78); color: #fff;
                    font: 700 15px/1.45 Consolas, "Courier New", monospace;
                    padding: 6px 10px; pointer-events: none;
                    border-bottom-right-radius: 6px; white-space: pre;
                }}
                #coord .t {{ color:#FF9500; font-size:12px; font-weight:400; }}
                #offline {{
                    display: none; position: fixed; inset: 0; z-index: 9998;
                    background: #1a1a1a; color: #fff; padding: 24px;
                    font: 400 14px/1.7 Consolas, "Courier New", monospace;
                }}
                #offline b {{ color:#FF3B30; font-size:18px; }}
                #offline .big {{ font-size: 30px; font-weight: 700; color:#00E676;
                                 letter-spacing: 1px; margin: 14px 0; }}
            </style>
        </head>
        <body>
            <div id="map"></div>
            <div id="coord">等待 GPS…</div>
            <div id="offline">
              <b>⚠ 地圖無法載入</b>（需要網際網路：unpkg.com 的 Leaflet + OSM 圖磚）<br>
              座標仍然正常更新，見下方與左上角。飛行資料完全不受影響。
              <div class="big" id="offbig">等待 GPS…</div>
              <div id="offlist" style="opacity:.75;font-size:12px;"></div>
            </div>
            <script>
            // ── 座標疊層：與 Leaflet 完全無關，先定義，確保它一定存在 ──
            var _pts = [];
            function _showCoord(lat, lng, timeStr) {{
                var t = timeStr ? '<span class="t">[' + timeStr + ']</span>\n' : '';
                var txt = lat.toFixed(6) + ', ' + lng.toFixed(6);
                document.getElementById('coord').innerHTML = t + txt;
                var ob = document.getElementById('offbig');
                if (ob) ob.textContent = txt;
                _pts.push((timeStr || '') + ' ' + txt);
                if (_pts.length > 12) _pts.shift();
                var ol = document.getElementById('offlist');
                if (ol) ol.textContent = _pts.slice().reverse().join('\n');
            }}
            // Leaflet 沒載進來 → 走純文字模式，並且【大聲說出來】
            if (typeof L === 'undefined') {{
                document.getElementById('offline').style.display = 'block';
                window.updateMarker  = function(lat, lng, follow, timeStr) {{ _showCoord(lat, lng, timeStr); }};
                window.addEventMarker = function() {{}};
            }} else {{
                // 初始中心點設在台灣 (縮放等級為 7，顯示全島概覽，無標記與軌跡)
                var map = L.map('map', {{ attributionControl: true }}).setView([{lat}, {lng}], 7);
                if (map.attributionControl) {{
                    map.attributionControl.setPrefix(false); // 移除預設的 Leaflet 外部超連結
                }}
                L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                    maxZoom: 19,
                    attribution: '&copy; OpenStreetMap'
                }}).addTo(map);

                var marker = null;
                var polyline = null;
                var pathCoords = [];
                var eventMarkers = [];

                function updateMarker(lat, lng, follow, timeStr) {{
                    var newLatLng = new L.LatLng(lat, lng);
                    pathCoords.push([lat, lng]);

                    if (marker === null) {{
                        // 首次收到定位：建立高亮主標示點與軌跡線 (取消彈出文字框)
                        marker = L.circleMarker(newLatLng, {{
                            radius: 7,
                            color: '#FFFFFF',
                            fillColor: '#FF3B30',
                            fillOpacity: 1.0,
                            weight: 2
                        }}).addTo(map);
                        
                        polyline = L.polyline(pathCoords, {{
                            color: '#FF3B30',
                            weight: 4,
                            opacity: 0.85
                        }}).addTo(map);
                        // 首次定位：縮放到詳細層級
                        map.setView(newLatLng, 15);
                    }} else {{
                        marker.setLatLng(newLatLng);
                        polyline.setLatLngs(pathCoords);
                        if (follow) {{
                            map.panTo(newLatLng);
                        }}
                    }}

                    // 新增歷史軌跡輕量點與 hover 時間提示 Tooltip
                    var pointTooltip = (timeStr ? "[" + timeStr + "] " : "") + lat.toFixed(5) + ", " + lng.toFixed(5);
                    L.circleMarker(newLatLng, {{
                        radius: 3,
                        color: '#FF3B30',
                        fillColor: '#FF9500',
                        fillOpacity: 0.7,
                        weight: 1
                    }}).bindTooltip(pointTooltip, {{ sticky: true }}).addTo(map);
                    _showCoord(lat, lng, timeStr);   /* 地圖正常時疊層照樣更新 */
                }}

                function addEventMarker(lat, lng, labelText, color) {{
                    var eventLatLng = new L.LatLng(lat, lng);
                    var markerColor = color || '#D500F9';
                    var m = L.circleMarker(eventLatLng, {{
                        radius: 9,
                        color: '#FFFFFF',
                        fillColor: markerColor,
                        fillOpacity: 0.9,
                        weight: 2
                    }}).bindPopup("<b>" + labelText + "</b>").addTo(map);
                    eventMarkers.push(m);
                }}
            }}   /* ← 對應上方 if (typeof L === 'undefined') 的 else */
            </script>
        </body>
        </html>
        """
        self.web_view.setHtml(html_content)
        self.map_initialized = True
        self.current_location = location
        
    def update(self, location: Tuple[float, float], follow: bool = True, time_str: str = ""):
        """
        更新位置標記與歷史軌跡。
        
        Args:
            location (Tuple[float, float]): 新的(緯度, 經度)位置
            follow (bool): True=鏡頭自動跟隨火箭，False=只更新標記不移動視角
            time_str (str): 可選的時間戳字串 (HH:MM:SS)
        """
        if location != self.current_location:
            self.current_location = location
            if self.map_initialized:
                lat, lng = location
                follow_js = "true" if follow else "false"
                # 與 add_event_marker 一樣做轉義：time_str 目前是遙測來的
                # HH:MM:SS，但它是【外部輸入】，不該直接內插進 JS 字串。
                safe_t = (time_str.replace("\\", "\\\\")
                                  .replace("'", "\\'").replace("\n", ""))
                time_js = f"'{safe_t}'" if time_str else "''"
                js_code = f"if (typeof updateMarker === 'function') {{ updateMarker({lat}, {lng}, {follow_js}, {time_js}); }}"
                self.web_view.page().runJavaScript(js_code)
            else:
                self.create_map(location)

    def add_event_marker(self, location: Tuple[float, float], label_text: str, color: str = "#D500F9"):
        """在地圖指定經緯度加上事件卡片標記"""
        if self.map_initialized and location:
            lat, lng = location
            # 轉義引號，防止 JS 字串截斷
            safe_label = label_text.replace("\\", "\\\\").replace("'", "\\'").replace('"', '\\"')
            js_code = f"if (typeof addEventMarker === 'function') {{ addEventMarker({lat}, {lng}, '{safe_label}', '{color}'); }}"
            self.web_view.page().runJavaScript(js_code)

    def reset(self, initial_location: Tuple[float, float] = (23.5, 121.5)):
        """重置地圖與歷史軌跡線標記"""
        self.create_map(initial_location)
