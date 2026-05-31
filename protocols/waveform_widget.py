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

        self.annotation_container = QWidget()
        self.annotation_layout = QVBoxLayout()
        self.annotation_layout.setContentsMargins(0, 0, 0, 0)
        self.annotation_container.setLayout(self.annotation_layout)
        self.annotation_container.hide()
        self.splitter.addWidget(self.annotation_container)

        self.splitter.setSizes([1, 0])

        self.vline = InfiniteLine(angle=90, movable=False, pen=pg.mkPen('r', width=1, style=Qt.DashLine))
        self.graph_widget.addItem(self.vline)
        self.vline.hide()

        self.reference_line = InfiniteLine(angle=90, movable=False, pen=pg.mkPen('b', width=1, style=Qt.DashLine))
        self.graph_widget.addItem(self.reference_line)
        self.reference_line.hide()

        self.curves = []
        self.reference_time = None

        self.decoder_plots = {}  # key: instance_id

        self.graph_widget.scene().sigMouseMoved.connect(self.on_mouse_move)
        self.graph_widget.scene().sigMouseClicked.connect(self.on_mouse_click)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_splitter_sizes()

    def plot_signals(self, samples, time_axis, config):
        if samples is None or time_axis is None:
            return
        for c in self.curves:
            self.graph_widget.removeItem(c)
        self.curves.clear()

        show = config.show_channels
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
            gpio = config.gpio.get(ch, -1)
            label = f"GPIO{gpio}" if gpio != -1 else f"CH{ch}"
            ticks.append((label_pos[idx], label))
        self.graph_widget.getAxis('left').setTicks([ticks])

        self.graph_widget.setXRange(time_axis[0], time_axis[-1])
        self.vline.hide()
        self.reference_line.hide()
        self.reference_time = None

    def clear_all_annotations(self):
        for inst_id in list(self.decoder_plots.keys()):
            self.remove_decoder_plot(inst_id)

    def add_decoder_plot(self, instance_id, proto_name):
        if instance_id in self.decoder_plots:
            return self.decoder_plots[instance_id]['plot']

        plot = pg.PlotWidget()
        short_id = instance_id[:8]
        plot.setLabel('left', f"{proto_name} ({short_id})")
        plot.showAxis('left', True)
        plot.showGrid(x=True, y=False, alpha=0.3)
        plot.setMouseEnabled(x=True, y=False)
        plot.setXLink(self.graph_widget)

        self.decoder_plots[instance_id] = {
            'plot': plot,
            'annotation_items': [],
            'text_items': [],
            'text_annotations': [],
            'font': QFont('sans-serif', 9),
            'font_metrics': QFontMetrics(QFont('sans-serif', 9)),
            'update_timer': QTimer(),
            'cached_sec_per_px': None,
            'samplerate': 1.0,
            'info': None,
            'out_map': None,
        }
        timer = self.decoder_plots[instance_id]['update_timer']
        timer.setSingleShot(True)
        timer.setInterval(100)
        timer.timeout.connect(lambda iid=instance_id: self._on_timer_timeout(iid))

        # Исправленные лямбды, игнорирующие аргументы сигналов
        plot.plotItem.vb.sigRangeChanged.connect(lambda *args, iid=instance_id: self._schedule_text_update(iid))
        plot.plotItem.vb.sigResized.connect(lambda *args, iid=instance_id: self._on_resized(iid))

        self.annotation_layout.addWidget(plot)
        self._update_splitter_sizes()
        return plot

    def remove_decoder_plot(self, instance_id):
        if instance_id not in self.decoder_plots:
            return
        data = self.decoder_plots[instance_id]
        for item in data['annotation_items']:
            data['plot'].removeItem(item)
        for item in data['text_items']:
            data['plot'].removeItem(item)
        self.annotation_layout.removeWidget(data['plot'])
        data['plot'].deleteLater()
        del self.decoder_plots[instance_id]
        self._update_splitter_sizes()

    def _update_splitter_sizes(self):
        num_decoders = len(self.decoder_plots)
        total_height = self.height()
        if num_decoders == 0 or total_height == 0:
            self.annotation_container.hide()
            self.splitter.setSizes([1, 0])
            return
        if not self.annotation_container.isVisible():
            self.annotation_container.show()
        single_annot_height = int(total_height * 0.2)
        desired_annot_height = single_annot_height * num_decoders
        max_annot_height = int(total_height * 0.7)
        annot_height = min(desired_annot_height, max_annot_height)
        signal_height = total_height - annot_height
        self.splitter.setSizes([signal_height, annot_height])

    def update_decoder_annotations(self, instance_id, results, out_map, info, samplerate):
        if instance_id not in self.decoder_plots:
            self.add_decoder_plot(instance_id, info.name)
        data = self.decoder_plots[instance_id]
        plot = data['plot']
        for item in data['annotation_items']:
            plot.removeItem(item)
        data['annotation_items'].clear()
        for item in data['text_items']:
            plot.removeItem(item)
        data['text_items'].clear()
        data['text_annotations'].clear()
        plot.getAxis('left').setTicks([])

        if not results:
            return

        data['samplerate'] = samplerate
        data['info'] = info
        data['out_map'] = out_map

        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
                  '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
        ann_color_map = {}
        if info.annotation_rows:
            for row_idx, row in enumerate(info.annotation_rows):
                _, _, indices = row
                for i in indices:
                    ann_color_map[i] = colors[i % len(colors)] if i != 4 else '#d62728'

        def get_color(ann_class):
            return ann_color_map.get(ann_class, colors[ann_class % len(colors)])

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

        for start, end, out_id, ann_data in results:
            if out_map.get(out_id) != 'ann':
                continue
            if not (isinstance(ann_data, list) and len(ann_data) >= 2):
                continue
            ann_class = ann_data[0]
            ann_texts = ann_data[1]
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
        plot.setYRange(0, total_height)

        tick_labels = []
        for i, original_row in enumerate(used_rows):
            name = next((n for orig, n in row_names if orig == original_row), "?")
            y_pos = i * row_height + row_height / 2.0
            tick_labels.append((y_pos, name))
        plot.getAxis('left').setTicks([tick_labels])

        data['text_annotations'] = []

        self._draw_interval_annotations(instance_id, rows_data, used_rows, row_height)
        self._draw_point_events(instance_id, point_events, used_rows, row_height)

        QTimer.singleShot(0, lambda: self._create_text_items(instance_id))

    def _draw_interval_annotations(self, instance_id, rows_data, used_rows, row_height):
        data = self.decoder_plots[instance_id]
        plot = data['plot']
        for actual_row, original_row in enumerate(used_rows):
            anns = rows_data.get(original_row, [])
            anns.sort(key=lambda x: x[0])
            y_base = actual_row * row_height
            prev_end = None
            for t_start, t_end, qcolor, ann_texts in anns:
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
                    plot.addItem(sep_line)
                    data['annotation_items'].append(sep_line)

                slant = min(0.05 * row_height, width * 0.05)
                x_p = [t_start, t_start+slant, t_end-slant, t_end, t_end-slant, t_start+slant, t_start]
                y_p = [y_base+row_height/2, y_base+row_height, y_base+row_height,
                       y_base+row_height/2, y_base, y_base, y_base+row_height/2]
                fill_color = QColor(qcolor)
                fill_color.setAlpha(100)
                border_color = QColor(qcolor).darker(120)
                polygon = pg.PlotDataItem(x_p, y_p, pen=pg.mkPen(border_color, width=1),
                                          brush=pg.mkBrush(fill_color))
                plot.addItem(polygon)
                data['annotation_items'].append(polygon)
                data['text_annotations'].append((t_start, t_end, ann_texts, actual_row, qcolor, False))
                prev_end = t_end

    def _draw_point_events(self, instance_id, point_events, used_rows, row_height):
        data = self.decoder_plots[instance_id]
        plot = data['plot']
        marker_size_px = 10
        for t_center, orig_row, qcolor, ann_texts in point_events:
            actual_row = used_rows.index(orig_row) if orig_row in used_rows else 0
            y_base = actual_row * row_height
            path = QPainterPath()
            pts = []
            for i in range(6):
                angle = np.pi/2 - i * np.pi/3
                pts.append(QPointF(np.cos(angle)*marker_size_px, -np.sin(angle)*marker_size_px))
            poly = QPolygonF(pts)
            path.addPolygon(poly)
            item = QGraphicsPathItem(path)
            item.setPen(QPen(qcolor, 2))
            item.setBrush(QBrush(qcolor.lighter(150)))
            item.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
            item.setPos(t_center, y_base + row_height/2)
            plot.addItem(item)
            data['annotation_items'].append(item)

            short_text = sorted(ann_texts, key=len)[0]
            one_char = short_text[0] if short_text else ''
            data['text_annotations'].append((t_center, t_center, [one_char], actual_row, qcolor, True))

    def _create_text_items(self, instance_id):
        data = self.decoder_plots[instance_id]
        plot = data['plot']
        for item in data['text_items']:
            plot.removeItem(item)
        data['text_items'].clear()

        vb = plot.plotItem.vb
        view_range = vb.viewRange()
        x_min, x_max = view_range[0]
        view_width_sec = x_max - x_min
        widget_width_px = vb.width()
        if widget_width_px == 0 or view_width_sec == 0:
            QTimer.singleShot(50, lambda: self._create_text_items(instance_id))
            return
        sec_per_px = view_width_sec / widget_width_px
        row_height = 2.0
        fm = data['font_metrics']

        for (t_start, t_end, ann_texts, row, qcolor, is_point) in data['text_annotations']:
            width = t_end - t_start
            width_px = width / sec_per_px if width > 0 else 0

            if is_point:
                displayed_text = ann_texts[0] if ann_texts else ''
            else:
                if len(ann_texts[0]) == 1:
                    displayed_text = ann_texts[0]
                else:
                    text = self._choose_text(ann_texts, width_px, fm)
                    displayed_text = self._truncate_text(text, width_px, fm)

            y_pos = row * row_height + row_height / 2.0
            text_item = pg.TextItem(displayed_text if displayed_text else '', anchor=(0.5, 0.5), color=qcolor.name())
            text_item.setFont(data['font'])
            text_item.setPos((t_start + t_end) / 2, y_pos)
            plot.addItem(text_item)
            data['text_items'].append(text_item)

    def _update_text_items(self, instance_id):
        data = self.decoder_plots[instance_id]
        plot = data['plot']
        vb = plot.plotItem.vb
        view_range = vb.viewRange()
        x_min, x_max = view_range[0]
        view_width_sec = x_max - x_min
        widget_width_px = vb.width()
        if widget_width_px == 0 or view_width_sec == 0:
            return
        sec_per_px = view_width_sec / widget_width_px
        fm = data['font_metrics']

        for i, (t_start, t_end, ann_texts, row, qcolor, is_point) in enumerate(data['text_annotations']):
            if i >= len(data['text_items']):
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
                    text = self._choose_text(ann_texts, width_px, fm)
                    displayed_text = self._truncate_text(text, width_px, fm)

            data['text_items'][i].setText(displayed_text if displayed_text else '')

    def _choose_text(self, texts, max_width_px, fm):
        if not texts:
            return ''
        sorted_texts = sorted(texts, key=len)
        for text in reversed(sorted_texts):
            if fm.width(text) <= max_width_px:
                return text
        return sorted_texts[0]

    def _truncate_text(self, text, max_width_px, fm):
        if not text or max_width_px <= 0:
            return ''
        if fm.width(text) <= max_width_px:
            return text
        max_chars = len(text)
        while max_chars > 0 and fm.width(text[:max_chars]) > max_width_px:
            max_chars -= 1
        return text[:max_chars] if max_chars > 0 else ''

    def _schedule_text_update(self, instance_id):
        data = self.decoder_plots[instance_id]
        if data['text_annotations'] and not data['update_timer'].isActive():
            data['update_timer'].start()

    def _on_resized(self, instance_id):
        data = self.decoder_plots[instance_id]
        data['cached_sec_per_px'] = None
        self._schedule_text_update(instance_id)

    def _on_timer_timeout(self, instance_id):
        data = self.decoder_plots[instance_id]
        vb = data['plot'].plotItem.vb
        view_range = vb.viewRange()
        x_min, x_max = view_range[0]
        view_width_sec = x_max - x_min
        widget_width_px = vb.width()
        if widget_width_px == 0 or view_width_sec == 0:
            return
        new_sec_per_px = view_width_sec / widget_width_px

        if data['cached_sec_per_px'] is None:
            data['cached_sec_per_px'] = new_sec_per_px
            self._update_text_items(instance_id)
        else:
            change = abs(new_sec_per_px - data['cached_sec_per_px']) / data['cached_sec_per_px']
            if change > 0.05:
                data['cached_sec_per_px'] = new_sec_per_px
                self._update_text_items(instance_id)

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