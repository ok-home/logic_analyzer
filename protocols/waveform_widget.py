# waveform_widget.py
import numpy as np
import pyqtgraph as pg
from pyqtgraph import InfiniteLine
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QSplitter, QApplication,
    QGraphicsPathItem, QGraphicsItem
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QPointF
from PyQt5.QtGui import (
    QFont, QFontMetrics, QPainterPath, QPolygonF, QPen, QBrush, QColor
)

class WaveformWidget(QWidget):
    reference_marker_changed = pyqtSignal(object)
    mouse_moved_with_time = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout()
        self.setLayout(layout)

        self.splitter = QSplitter(Qt.Vertical)
        layout.addWidget(self.splitter)

        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setLabel('bottom', 'Time', 's')
        self.graph_widget.setLabel('left', 'Channels')
        self.graph_widget.showGrid(x=True, y=False, alpha=0.5)
        self.graph_widget.setMouseEnabled(x=True, y=False)
        self.splitter.addWidget(self.graph_widget)

        self.annotation_plot = pg.PlotWidget()
        self.annotation_plot.setLabel('bottom', 'Time', 's')
        self.annotation_plot.setLabel('left', 'Annotations')
        self.annotation_plot.showAxis('left', True)
        self.annotation_plot.showGrid(x=True, y=False, alpha=0.3)
        self.annotation_plot.setMouseEnabled(x=True, y=False)
        self.annotation_plot.hide()
        self.splitter.addWidget(self.annotation_plot)

        self.annotation_plot.setXLink(self.graph_widget)
        self.splitter.setSizes([500, 200])

        self.vline = InfiniteLine(angle=90, movable=False, pen=pg.mkPen('r', width=1, style=Qt.DashLine))
        self.graph_widget.addItem(self.vline)
        self.vline.hide()

        self.reference_line = InfiniteLine(angle=90, movable=False, pen=pg.mkPen('b', width=1, style=Qt.DashLine))
        self.graph_widget.addItem(self.reference_line)
        self.reference_line.hide()

        self.curves = []
        self.annotation_items = []
        self.text_items = []
        self.text_annotations = []  # (t_start, t_end, ann_texts, row, color, is_point_event)
        self.reference_time = None

        self.text_font = QFont('sans-serif', 9)
        self.font_metrics = QFontMetrics(self.text_font)

        self._cached_sec_per_px = None
        self.update_timer = QTimer()
        self.update_timer.setSingleShot(True)
        self.update_timer.setInterval(100)
        self.update_timer.timeout.connect(self._on_timer_timeout)

        self.annotation_plot.plotItem.vb.sigRangeChanged.connect(self._on_range_changed)

        self.graph_widget.scene().sigMouseMoved.connect(self.on_mouse_move)
        self.graph_widget.scene().sigMouseClicked.connect(self.on_mouse_click)

    # ---------- сигналы ----------
    def plot_signals(self, samples, time_axis, config):
        if samples is None or time_axis is None:
            return
        for c in self.curves:
            self.graph_widget.removeItem(c)
        self.curves.clear()
        self.clear_annotations()

        show = config.get('show_channels', list(range(16)))
        if not show:
            return

        ch_height = 1.2
        sig_height = 1.0
        top_margin = (ch_height - sig_height) / 2.0
        y_offsets = np.arange(len(show)-1, -1, -1) * ch_height

        dt = time_axis[1] - time_axis[0] if len(time_axis) > 1 else 1.0
        x_step = np.append(time_axis, time_axis[-1] + dt)

        for idx, ch in enumerate(show):
            y_vals = samples[ch, :] * sig_height + y_offsets[idx] + top_margin
            curve = self.graph_widget.plot(x_step, y_vals, stepMode=True, pen=pg.mkPen('g', width=1))
            self.curves.append(curve)

        y_min = -ch_height * 0.2
        y_max = len(show) * ch_height + ch_height * 0.2
        self.graph_widget.setYRange(y_min, y_max)

        label_pos = y_offsets + ch_height / 2.0
        ticks = []
        for idx, ch in enumerate(show):
            gpio = config.get('gpio', {}).get(ch, '-1')
            label = f"GPIO{gpio}" if gpio != '-1' else f"CH{ch}"
            ticks.append((label_pos[idx], label))
        self.graph_widget.getAxis('left').setTicks([ticks])

        self.graph_widget.setXRange(time_axis[0], time_axis[-1])
        self.vline.hide()
        self.reference_line.hide()
        self.reference_time = None

    def clear_annotations(self):
        for item in self.annotation_items:
            self.annotation_plot.removeItem(item)
        self.annotation_items.clear()
        for item in self.text_items:
            self.annotation_plot.removeItem(item)
        self.text_items.clear()
        self.text_annotations.clear()
        self.annotation_plot.getAxis('left').setTicks([])
        self.annotation_plot.hide()

    def add_annotations(self, results, out_map, info, samplerate):
        self.clear_annotations()

        # Расширенная палитра (можно добавлять новые цвета)
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
                  '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']

        ann_color_map = {}
        if info.annotation_rows:
            for row_idx, row in enumerate(info.annotation_rows):
                _, _, indices = row
                for i in indices:
                    if i == 4:   # предупреждения (UART)
                        ann_color_map[i] = '#d62728'
                    else:
                        # Для всех остальных аннотаций используем общую палитру
                        ann_color_map[i] = colors[i % len(colors)]

        # Если для какого-то класса нет цвета, добавляем fallback
        def get_color(ann_class):
            if ann_class in ann_color_map:
                return ann_color_map[ann_class]
            # Fallback: используем цвет по индексу класса
            return colors[ann_class % len(colors)]

        row_assign = {}
        row_names = []
        if info.annotation_rows:
            for row_idx, row in enumerate(info.annotation_rows):
                name, _, indices = row
                row_names.append((row_idx, name))
                for ann_idx in indices:
                    row_assign[ann_idx] = row_idx

        num_rows_total = len(info.annotation_rows) if info.annotation_rows else 1
        rows_data = {r: [] for r in range(num_rows_total)}
        point_events = []

        for start, end, out_id, data in results:
            if out_map.get(out_id) != 'ann':
                continue
            if not (isinstance(data, list) and len(data) >= 2):
                continue
            ann_class = data[0]
            ann_texts = data[1]
            t_start = start / samplerate
            t_end = end / samplerate
            row = row_assign.get(ann_class, 0)
            color_str = get_color(ann_class)
            try:
                qcolor = QColor(color_str)
            except:
                qcolor = QColor('#aaa')
            if t_end == t_start:
                point_events.append((t_start, row, qcolor, ann_texts))
            else:
                rows_data[row].append((t_start, t_end, qcolor, ann_texts))

        used_rows = [r for r in range(num_rows_total) if rows_data.get(r) or any(ev[1] == r for ev in point_events)]
        if not used_rows:
            return

        row_height = 2.0
        total_height = len(used_rows) * row_height
        self.annotation_plot.setYRange(0, total_height)
        self.annotation_plot.show()

        tick_labels = []
        for i, original_row in enumerate(used_rows):
            name = "?"
            for orig_idx, n in row_names:
                if orig_idx == original_row:
                    name = n
                    break
            y_pos = i * row_height + row_height / 2.0
            tick_labels.append((y_pos, name))
        self.annotation_plot.getAxis('left').setTicks([tick_labels])

        self.text_annotations = []

        # Обычные полигоны
        for actual_row, original_row in enumerate(used_rows):
            anns = rows_data.get(original_row, [])
            anns.sort(key=lambda x: x[0])

            y_base = actual_row * row_height
            prev_end = None
            for (t_start, t_end, qcolor, ann_texts) in anns:
                width = t_end - t_start
                if width <= 0:
                    continue

                if prev_end is not None and t_start > prev_end:
                    sep_line = pg.PlotDataItem(
                        [t_start, t_start],
                        [y_base, y_base + row_height],
                        connect='pairs',
                        pen=pg.mkPen('k', width=1, style=Qt.DashLine)
                    )
                    self.annotation_plot.addItem(sep_line)
                    self.annotation_items.append(sep_line)

                slant = min(0.05 * row_height, width * 0.05)
                x_p = [
                    t_start,
                    t_start + slant,
                    t_end - slant,
                    t_end,
                    t_end - slant,
                    t_start + slant,
                    t_start
                ]
                y_p = [
                    y_base + row_height/2,
                    y_base + row_height,
                    y_base + row_height,
                    y_base + row_height/2,
                    y_base,
                    y_base,
                    y_base + row_height/2
                ]
                fill_color = QColor(qcolor)
                fill_color.setAlpha(100)
                border_color = QColor(qcolor).darker(120)
                polygon = pg.PlotDataItem(
                    x_p, y_p,
                    pen=pg.mkPen(border_color, width=1),
                    brush=pg.mkBrush(fill_color)
                )
                self.annotation_plot.addItem(polygon)
                self.annotation_items.append(polygon)

                self.text_annotations.append((t_start, t_end, ann_texts, actual_row, qcolor, False))
                prev_end = t_end

        self.annotation_plot.setXRange(*self.graph_widget.viewRange()[0])
        QApplication.processEvents()

        # Точечные события: фиксированный размер в пикселях
        marker_size_px = 10
        for (t_center, orig_row, qcolor, ann_texts) in point_events:
            actual_row = used_rows.index(orig_row) if orig_row in used_rows else 0
            y_base = actual_row * row_height
            path = QPainterPath()
            pts = []
            for i in range(6):
                angle = np.pi/2 - i * np.pi/3
                pts.append(QPointF(np.cos(angle) * marker_size_px, -np.sin(angle) * marker_size_px))
            poly = QPolygonF(pts)
            path.addPolygon(poly)
            item = QGraphicsPathItem(path)
            item.setPen(QPen(qcolor, 2))
            item.setBrush(QBrush(qcolor.lighter(150)))
            item.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            item.setPos(t_center, y_base + row_height/2)
            self.annotation_plot.addItem(item)
            self.annotation_items.append(item)

            short_text = sorted(ann_texts, key=len)[0]
            one_char = short_text[0] if short_text else ''
            self.text_annotations.append((t_center, t_center, [one_char], actual_row, qcolor, True))

        self._create_text_items()

    def _choose_text(self, texts, max_width_px):
        if not texts:
            return ''
        sorted_texts = sorted(texts, key=len)
        for text in reversed(sorted_texts):
            if self.font_metrics.width(text) <= max_width_px:
                return text
        return sorted_texts[0]

    def _truncate_text(self, text, max_width_px):
        if not text or max_width_px <= 0:
            return ''
        if self.font_metrics.width(text) <= max_width_px:
            return text
        max_chars = len(text)
        while max_chars > 0 and self.font_metrics.width(text[:max_chars]) > max_width_px:
            max_chars -= 1
        return text[:max_chars] if max_chars > 0 else ''

    def _create_text_items(self):
        for item in self.text_items:
            self.annotation_plot.removeItem(item)
        self.text_items.clear()

        vb = self.annotation_plot.plotItem.vb
        view_range = vb.viewRange()
        x_min, x_max = view_range[0]
        view_width_sec = x_max - x_min
        widget_width_px = vb.width()
        if widget_width_px == 0 or view_width_sec == 0:
            QTimer.singleShot(50, self._create_text_items)
            return
        sec_per_px = view_width_sec / widget_width_px
        row_height = 2.0

        for (t_start, t_end, ann_texts, row, qcolor, is_point) in self.text_annotations:
            width = t_end - t_start
            width_px = width / sec_per_px if width > 0 else 0

            if is_point:
                displayed_text = ann_texts[0] if ann_texts else ''
            else:
                if len(ann_texts[0]) == 1:
                    displayed_text = ann_texts[0]
                else:
                    text = self._choose_text(ann_texts, width_px)
                    displayed_text = self._truncate_text(text, width_px)

            y_pos = row * row_height + row_height / 2.0
            text_item = pg.TextItem(displayed_text if displayed_text else '', anchor=(0.5, 0.5), color=qcolor.name())
            text_item.setFont(self.text_font)
            text_item.setPos((t_start + t_end) / 2, y_pos)
            self.annotation_plot.addItem(text_item)
            self.text_items.append(text_item)

    def _update_text_items(self):
        vb = self.annotation_plot.plotItem.vb
        view_range = vb.viewRange()
        x_min, x_max = view_range[0]
        view_width_sec = x_max - x_min
        widget_width_px = vb.width()
        if widget_width_px == 0 or view_width_sec == 0:
            return
        sec_per_px = view_width_sec / widget_width_px

        for i, (t_start, t_end, ann_texts, row, qcolor, is_point) in enumerate(self.text_annotations):
            if i >= len(self.text_items):
                continue
            if not is_point and (t_end < x_min or t_start > x_max):
                continue

            width = t_end - t_start
            width_px = width / sec_per_px if width > 0 else 0

            if is_point:
                displayed_text = ann_texts[0] if ann_texts else ''
            else:
                if len(ann_texts[0]) == 1:
                    displayed_text = ann_texts[0]
                else:
                    text = self._choose_text(ann_texts, width_px)
                    displayed_text = self._truncate_text(text, width_px)

            self.text_items[i].setText(displayed_text if displayed_text else '')

    def _on_range_changed(self, vb, ranges):
        if self.text_annotations:
            if not self.update_timer.isActive():
                self.update_timer.start()

    def _on_timer_timeout(self):
        vb = self.annotation_plot.plotItem.vb
        view_range = vb.viewRange()
        x_min, x_max = view_range[0]
        view_width_sec = x_max - x_min
        widget_width_px = vb.width()
        if widget_width_px == 0 or view_width_sec == 0:
            return
        new_sec_per_px = view_width_sec / widget_width_px

        if self._cached_sec_per_px is None:
            self._cached_sec_per_px = new_sec_per_px
            self._update_text_items()
        else:
            change = abs(new_sec_per_px - self._cached_sec_per_px) / self._cached_sec_per_px
            if change > 0.05:
                self._cached_sec_per_px = new_sec_per_px
                self._update_text_items()

    # ---------- события мыши ----------
    def on_mouse_move(self, pos):
        vb = self.graph_widget.plotItem.vb
        if vb.sceneBoundingRect().contains(pos):
            mouse_point = vb.mapSceneToView(pos)
            t = mouse_point.x()
            self.mouse_moved_with_time.emit(t)
            xlim = self.graph_widget.viewRange()[0]
            if xlim[0] <= t <= xlim[1]:
                self.vline.setPos(t)
                self.vline.show()
            else:
                self.vline.hide()
        else:
            self.mouse_moved_with_time.emit(None)
            self.vline.hide()

    def on_mouse_click(self, event):
        if event.button() == Qt.LeftButton:
            vb = self.graph_widget.plotItem.vb
            pos = event.scenePos()
            if vb.sceneBoundingRect().contains(pos):
                mouse_point = vb.mapSceneToView(pos)
                t = mouse_point.x()
                self.reference_time = t
                self.reference_line.setPos(t)
                self.reference_line.show()
                self.reference_marker_changed.emit(t)
        elif event.button() == Qt.RightButton:
            self.clear_reference_marker()

    def clear_reference_marker(self):
        self.reference_time = None
        self.reference_line.hide()
        self.reference_marker_changed.emit(None)

    def get_reference_time(self):
        return self.reference_time

    def scene(self):
        return self.graph_widget.scene()