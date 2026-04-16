"""
M5Stack UDP通信 - リアルタイムプロッタ
=======================================
matplotlib を用いて受信データをリアルタイムにグラフ描画する。
デバイス1〜6のセンサ値 (CH1-3) を6つのサブプロットで表示。
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import logging

from config import NUM_DEVICES

logger = logging.getLogger(__name__)

# プロット設定
MAX_POINTS = 500  # 画面に表示する最大データ点数 (200Hzなら約2.5秒分)
UPDATE_INTERVAL_MS = 50  # 画面更新間隔 (50ms = 20fps)

class RealtimePlotter:
    def __init__(self):
        # 描画用データのバッファ: [デバイスID][チャンネルID] -> deque
        self.t_data = {dev: deque(maxlen=MAX_POINTS) for dev in range(1, NUM_DEVICES + 1)}
        self.y_data = {
            dev: {ch: deque(maxlen=MAX_POINTS) for ch in range(3)}  # CH1, CH2, CH3
            for dev in range(1, NUM_DEVICES + 1)
        }
        
        self.lock = threading.Lock()
        
        # グラフの設定
        self.fig, self.axes = plt.subplots(3, 2, figsize=(10, 8))
        self.fig.suptitle("Real-time Force Data (CH 10-12)")
        self.fig.tight_layout(pad=3.0)
        
        # 軸を平坦化してアクセスしやすくする
        self.axes_flat = self.axes.flatten()
        
        # 線のオブジェクトを保存
        self.lines = {}
        colors = ['red', 'green', 'blue']
        labels = ['Force X', 'Force Y', 'Force Z']
        
        for i in range(NUM_DEVICES):
            dev_id = i + 1
            ax = self.axes_flat[i]
            ax.set_title(f"Device {dev_id}")
            ax.set_ylim(-3.5, 3.5)  # 力データの想定範囲に合わせて調整
            ax.grid(True)
            
            self.lines[dev_id] = {}
            for ch in range(3):
                line, = ax.plot([], [], color=colors[ch], label=labels[ch])
                self.lines[dev_id][ch] = line
                
            ax.legend(loc="upper right", fontsize='small')

    def add_data(self, device_id: int, timestamp: float, ch1: float, ch2: float, ch3: float):
        """新しいデータをバッファに追加 (Consumerスレッドから呼ばれる)"""
        if device_id not in self.t_data:
            return
            
        with self.lock:
            self.t_data[device_id].append(timestamp)
            self.y_data[device_id][0].append(ch1)
            self.y_data[device_id][1].append(ch2)
            self.y_data[device_id][2].append(ch3)

    def _update_plot(self, frame):
        """アニメーション更新関数 (メインスレッドのFuncAnimationから呼ばれる)"""
        with self.lock:
            for dev_id in range(1, NUM_DEVICES + 1):
                if not self.t_data[dev_id]:
                    continue
                    
                t = list(self.t_data[dev_id])
                
                # X軸の設定 (最新のtから過去へ)
                ax = self.axes_flat[dev_id - 1]
                t_min = t[0]
                t_max = t[-1]
                if t_max - t_min > 0:
                    ax.set_xlim(t_min, t_max)
                    
                # 線の更新
                for ch in range(3):
                    y = list(self.y_data[dev_id][ch])
                    self.lines[dev_id][ch].set_data(t, y)
                    
        return [line for dev_lines in self.lines.values() for line in dev_lines.values()]

    def show(self):
        """グラフウィンドウを表示する (メインスレッドで実行する必要あり)"""
        logger.info("グラフウィンドウを起動します。終了するにはウィンドウを閉じてください。")
        self.ani = animation.FuncAnimation(
            self.fig, self._update_plot, interval=UPDATE_INTERVAL_MS, blit=False, cache_frame_data=False
        )
        # plt.show() はブロック関数。ウィンドウが閉じられるまで戻らない。
        plt.show()
