"""自動換行的水平佈局。

操作列(焦點鈕 + 六顆點火鈕 + 校準 + Auto)在 1200px 寬的視窗剛好排得下,
但比賽現場若換成小螢幕筆電就會溢出、把右側圖表擠掉。QHBoxLayout 不會換行,
只會把元件壓扁或撐爆版面;這個 layout 在寬度不足時自動折到下一列。

改寫自 Qt 官方 FlowLayout 範例。
"""
from PyQt6.QtCore import QPoint, QRect, QSize, Qt
from PyQt6.QtWidgets import QLayout, QSizePolicy


class FlowLayout(QLayout):
    def __init__(self, parent=None, margin: int = 0, spacing: int = 6):
        super().__init__(parent)
        self._items = []
        self.setContentsMargins(margin, margin, margin, margin)
        self.setSpacing(spacing)

    # ── QLayout 必要介面 ──────────────────────────────────────────
    def addItem(self, item):
        self._items.append(item)

    def count(self):
        return len(self._items)

    def itemAt(self, index):
        return self._items[index] if 0 <= index < len(self._items) else None

    def takeAt(self, index):
        return self._items.pop(index) if 0 <= index < len(self._items) else None

    def expandingDirections(self):
        return Qt.Orientation(0)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        return self._do_layout(QRect(0, 0, width, 0), test_only=True)

    def setGeometry(self, rect):
        super().setGeometry(rect)
        self._do_layout(rect, test_only=False)

    def sizeHint(self):
        return self.minimumSize()

    def minimumSize(self):
        """最小尺寸 = 最寬的那顆元件(而非全部相加)——這正是能換行的前提:
        視窗再窄也只要塞得下單顆按鈕即可,其餘自動折行。"""
        size = QSize()
        for item in self._items:
            size = size.expandedTo(item.minimumSize())
        margins = self.contentsMargins()
        return size + QSize(margins.left() + margins.right(),
                            margins.top() + margins.bottom())

    # ── 排版本體 ─────────────────────────────────────────────────
    def _do_layout(self, rect: QRect, test_only: bool) -> int:
        margins = self.contentsMargins()
        eff = rect.adjusted(margins.left(), margins.top(),
                            -margins.right(), -margins.bottom())
        x, y, line_height = eff.x(), eff.y(), 0
        space = self.spacing()

        for item in self._items:
            hint = item.sizeHint()
            next_x = x + hint.width() + space
            if next_x - space > eff.right() and line_height > 0:
                # 這一列放不下了 → 折到下一列
                x = eff.x()
                y = y + line_height + space
                next_x = x + hint.width() + space
                line_height = 0
            if not test_only:
                item.setGeometry(QRect(QPoint(x, y), hint))
            x = next_x
            line_height = max(line_height, hint.height())

        return y + line_height - rect.y() + margins.bottom()
