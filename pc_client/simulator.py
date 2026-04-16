"""
M5Stack UDP通信 - テスト用シミュレータ
=======================================
M5Stackの代わりにダミーデータを生成し、UDPで送信するシミュレータ。
実機がなくても受信側の動作確認が可能。

使い方:
    python simulator.py [--devices N] [--rate HZ] [--samples N]

例:
    python simulator.py                  # デフォルト (6台, 200Hz, 10サンプル/パケット)
    python simulator.py --devices 2      # 2台のみ
    python simulator.py --rate 100       # 100Hz
"""

import argparse
import math
import random
import socket
import struct
import threading
import time
import logging

from config import (
    DATA_PORT,
    COMMAND_PORT,
    HEADER_MARKER,
    FOOTER_MARKER,
    NUM_CHANNELS,
    SAMPLES_PER_PACKET,
    SAMPLING_RATE,
    CMD_START,
    CMD_STOP,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("simulator")


class M5StackSimulator:
    """M5Stack 1台分のシミュレータ"""

    def __init__(
        self,
        device_id: int,
        target_ip: str,
        target_port: int,
        sampling_rate: int,
        samples_per_packet: int,
    ):
        self.device_id = device_id
        self.target_ip = target_ip
        self.target_port = target_port
        self.sampling_rate = sampling_rate
        self.samples_per_packet = samples_per_packet
        self.sequence_no = 0
        self.timer_us = 0  # 内部タイマー (マイクロ秒)
        self._running = False
        self._thread: threading.Thread | None = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def start(self):
        """計測開始 (内部タイマーリセット)"""
        self.timer_us = 0
        self.sequence_no = 0
        self._running = True
        self._thread = threading.Thread(
            target=self._send_loop,
            name=f"Sim-Dev{self.device_id}",
            daemon=True,
        )
        self._thread.start()
        logger.info(f"Device {self.device_id}: シミュレーション開始")

    def stop(self):
        """計測停止"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        logger.info(
            f"Device {self.device_id}: シミュレーション停止 "
            f"(送信パケット数: {self.sequence_no})"
        )

    def _generate_sample(self) -> tuple[int, list[float]]:
        """
        ダミーサンプルを生成する。
        サイン波ベースのテストデータを返す。
        """
        t = self.timer_us / 1_000_000.0  # 秒に変換
        interval_us = 1_000_000 // self.sampling_rate
        self.timer_us += interval_us

        channels = []
        for ch in range(NUM_CHANNELS):
            freq = 1.0 + ch * 0.5  # チャンネルごとに異なる周波数
            amplitude = 10.0 + ch * 2.0
            noise = random.gauss(0, 0.1)
            value = amplitude * math.sin(2 * math.pi * freq * t) + noise
            channels.append(value)

        return self.timer_us - interval_us, channels

    def _build_packet(self, samples: list[tuple[int, list[float]]]) -> bytes:
        """パケットをバイナリに組み立てる"""
        buf = bytearray()

        # Header
        buf += struct.pack(">H", HEADER_MARKER)
        # Device ID
        buf += struct.pack("B", self.device_id)
        # Sequence No.
        buf += struct.pack(">I", self.sequence_no)

        # Payload
        for timestamp_us, channels in samples:
            buf += struct.pack(">I", timestamp_us)
            buf += struct.pack(f">{NUM_CHANNELS}f", *channels)

        # Footer
        buf += struct.pack(">H", FOOTER_MARKER)

        return bytes(buf)

    def _send_loop(self):
        """送信ループ"""
        interval = self.samples_per_packet / self.sampling_rate

        while self._running:
            # N サンプル分バッファリング
            samples = []
            for _ in range(self.samples_per_packet):
                samples.append(self._generate_sample())

            # パケット組み立て & 送信
            packet = self._build_packet(samples)
            self._sock.sendto(packet, (self.target_ip, self.target_port))

            self.sequence_no += 1
            time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(
        description="M5Stack UDP シミュレータ (テスト用)"
    )
    parser.add_argument(
        "--devices", type=int, default=6,
        help="シミュレートするデバイス数 (デフォルト: 6)",
    )
    parser.add_argument(
        "--rate", type=int, default=SAMPLING_RATE,
        help=f"サンプリングレート [Hz] (デフォルト: {SAMPLING_RATE})",
    )
    parser.add_argument(
        "--samples", type=int, default=SAMPLES_PER_PACKET,
        help=f"1パケットあたりのサンプル数 (デフォルト: {SAMPLES_PER_PACKET})",
    )
    parser.add_argument(
        "--target-ip", type=str, default="127.0.0.1",
        help="送信先IPアドレス (デフォルト: 127.0.0.1)",
    )
    parser.add_argument(
        "--target-port", type=int, default=DATA_PORT,
        help=f"送信先ポート (デフォルト: {DATA_PORT})",
    )
    args = parser.parse_args()

    print()
    print("=" * 60)
    print("  M5Stack UDP シミュレータ")
    print("=" * 60)
    print()
    print(f"  デバイス数      : {args.devices}")
    print(f"  サンプリングレート: {args.rate} Hz")
    print(f"  パッキング数    : {args.samples} samples/packet")
    print(f"  送信先          : {args.target_ip}:{args.target_port}")
    print()

    # --- コマンド待受 ---
    print("STARTコマンドを待機中...")
    print("(別ターミナルで main.py を起動し、start と入力してください)")
    print()

    cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cmd_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    cmd_sock.bind(("", COMMAND_PORT))

    simulators: list[M5StackSimulator] = []
    running = True

    while running:
        try:
            data, addr = cmd_sock.recvfrom(1024)
            command = data.strip()

            if command == CMD_START:
                logger.info(f"STARTコマンド受信 from {addr}")

                # 既存のシミュレータを停止
                for sim in simulators:
                    sim.stop()
                simulators.clear()

                # 送信先IPをSTARTコマンドの送信元に設定
                target_ip = addr[0]
                if target_ip == "0.0.0.0":
                    target_ip = "127.0.0.1"

                # シミュレータ起動
                for dev_id in range(1, args.devices + 1):
                    sim = M5StackSimulator(
                        device_id=dev_id,
                        target_ip=target_ip,
                        target_port=args.target_port,
                        sampling_rate=args.rate,
                        samples_per_packet=args.samples,
                    )
                    sim.start()
                    simulators.append(sim)

                print(f"✅ {args.devices}台のシミュレーション開始 → {target_ip}:{args.target_port}")

            elif command == CMD_STOP:
                logger.info(f"STOPコマンド受信 from {addr}")
                for sim in simulators:
                    sim.stop()
                simulators.clear()
                print("✅ シミュレーション停止")

        except KeyboardInterrupt:
            print("\n⏹ シミュレータを終了します")
            for sim in simulators:
                sim.stop()
            running = False

    cmd_sock.close()


if __name__ == "__main__":
    main()
