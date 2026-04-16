"""
M5Stack UDP通信 - データ保存モジュール
=======================================
受信データを CSV ファイルに保存する。
デバイスごとに別ファイルとして出力する。
"""

import csv
import os
import logging
from datetime import datetime
from typing import Optional

from config import OUTPUT_DIR, NUM_DEVICES, CHANNEL_NAMES, SAMPLES_PER_PACKET
from packet import DecodedPacket

logger = logging.getLogger(__name__)


class DataSaver:
    """
    デバイス別データを CSV に保存するクラス。

    出力ファイル名:
        data/{YYYYMMDD_HHMMSS}/device_{id}_{YYYYMMDD_HHMMSS}.csv

    CSV カラム:
        packet_seq, sample_idx, timestamp_us, timestamp_adjusted_s,
        sensor_1, sensor_2, sensor_3,
        accel_x, accel_y, accel_z,
        gyro_x, gyro_y, gyro_z,
        load_1, load_2, load_3
    """

    def __init__(self, output_dir: str = OUTPUT_DIR):
        self.output_dir = output_dir

    def save_all_devices(
        self,
        device_data: dict[int, list[DecodedPacket]],
        measurement_start_time: Optional[float] = None,
    ) -> list[str]:
        """
        全デバイスのデータを CSV に保存する。

        Parameters
        ----------
        device_data : dict
            デバイスID → DecodedPacket のリスト
        measurement_start_time : float, optional
            計測開始時のPC側絶対時刻 (time.time())。
            指定された場合、相対タイムスタンプに加算して絶対時刻を計算する。

        Returns
        -------
        list[str]
            保存したファイルパスのリスト
        """
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = os.path.join(self.output_dir, timestamp_str)
        os.makedirs(session_dir, exist_ok=True)
        saved_files: list[str] = []

        for dev_id in range(1, NUM_DEVICES + 1):
            packets = device_data.get(dev_id, [])
            if not packets:
                logger.info(f"Device {dev_id}: データなし (スキップ)")
                continue

            filename = f"device_{dev_id}_{timestamp_str}.csv"
            filepath = os.path.join(session_dir, filename)

            self._save_device_csv(filepath, dev_id, packets, measurement_start_time)
            saved_files.append(filepath)
            total_samples = sum(len(p.samples) for p in packets)
            logger.info(
                f"Device {dev_id}: {len(packets)} パケット, "
                f"{total_samples} サンプル → {filepath}"
            )

        return saved_files

    def _save_device_csv(
        self,
        filepath: str,
        device_id: int,
        packets: list[DecodedPacket],
        measurement_start_time: Optional[float],
    ):
        """1デバイス分のデータをCSVに書き出す"""

        # ヘッダ行
        header = ["packet_seq", "sample_idx", "timestamp_us"]
        if measurement_start_time is not None:
            header.append("timestamp_adjusted_s")
        header.extend(CHANNEL_NAMES)

        with open(filepath, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)

            for pkt in packets:
                for idx, sample in enumerate(pkt.samples):
                    row = [pkt.sequence_no, idx, sample.timestamp_us]

                    # 絶対時刻の計算 (相対時間 + PC計測開始時刻)
                    if measurement_start_time is not None:
                        adjusted = (
                            measurement_start_time
                            + sample.timestamp_us / 1_000_000.0
                        )
                        row.append(f"{adjusted:.6f}")

                    row.extend(sample.channels)
                    writer.writerow(row)
