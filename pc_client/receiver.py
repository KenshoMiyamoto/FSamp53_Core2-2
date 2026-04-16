"""
M5Stack UDP通信 - 受信サーバ (Producer-Consumer パターン)
========================================================
Producer スレッド: UDP ソケットからデータを受信し、Queue に投入する。
Consumer スレッド: Queue からデータを取り出し、デコード・デバイス別振り分けを行う。
"""

import socket
import threading
import time
import logging
from queue import Queue, Empty
from typing import Callable, Optional

from config import (
    DATA_PORT,
    UDP_RECV_BUFFER_SIZE,
    QUEUE_MAX_SIZE,
    PACKET_SIZE,
    NUM_DEVICES,
)
from packet import decode_packet, DecodedPacket, PacketDecodeError, validate_packet_quick

logger = logging.getLogger(__name__)


class UDPReceiver:
    """
    UDP データ受信サーバ。
    Producer-Consumer パターンで受信とデコードを分離する。

    Attributes
    ----------
    port : int
        受信ポート番号
    packet_queue : Queue
        受信パケットの中間バッファ
    device_data : dict[int, list[DecodedPacket]]
        デバイスID別に振り分けられたデコード済みデータ
    last_seq : dict[int, int]
        デバイスIDごとの最後のシーケンス番号 (パケットロス検知用)
    lost_count : dict[int, int]
        デバイスIDごとのロストパケット数
    """

    def __init__(
        self,
        port: int = DATA_PORT,
        on_packet_decoded: Optional[Callable[[DecodedPacket], None]] = None,
    ):
        """
        Parameters
        ----------
        port : int
            データ受信ポート番号
        on_packet_decoded : callable, optional
            パケットデコード完了時のコールバック関数
        """
        self.port = port
        self.packet_queue: Queue = Queue(maxsize=QUEUE_MAX_SIZE)
        self.on_packet_decoded = on_packet_decoded

        # デバイス別データ格納
        self.device_data: dict[int, list[DecodedPacket]] = {
            i: [] for i in range(1, NUM_DEVICES + 1)
        }
        # パケットロス検知
        self.last_seq: dict[int, int] = {
            i: -1 for i in range(1, NUM_DEVICES + 1)
        }
        self.lost_count: dict[int, int] = {
            i: 0 for i in range(1, NUM_DEVICES + 1)
        }
        # 統計情報
        self.total_received = 0
        self.total_decoded = 0
        self.total_errors = 0

        # スレッド制御
        self._running = False
        self._sock: Optional[socket.socket] = None
        self._producer_thread: Optional[threading.Thread] = None
        self._consumer_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def start(self):
        """受信を開始する (Producer / Consumer スレッドを起動)"""
        if self._running:
            logger.warning("受信サーバは既に動作中です")
            return

        # --- ソケットの作成と設定 ---
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # OSバッファの拡張 (1MB以上)
        self._sock.setsockopt(
            socket.SOL_SOCKET, socket.SO_RCVBUF, UDP_RECV_BUFFER_SIZE
        )
        actual_buf = self._sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        logger.info(
            f"UDP受信バッファ: 要求={UDP_RECV_BUFFER_SIZE // 1024}KB, "
            f"実際={actual_buf // 1024}KB"
        )

        self._sock.bind(("", self.port))
        self._sock.settimeout(0.5)  # recvfrom のタイムアウト

        self._running = True

        # --- Producer スレッド ---
        self._producer_thread = threading.Thread(
            target=self._producer_loop,
            name="UDP-Producer",
            daemon=True,
        )
        # --- Consumer スレッド ---
        self._consumer_thread = threading.Thread(
            target=self._consumer_loop,
            name="UDP-Consumer",
            daemon=True,
        )

        self._producer_thread.start()
        self._consumer_thread.start()
        logger.info(f"UDP受信サーバ起動: ポート {self.port}")

    def stop(self):
        """受信を停止する"""
        self._running = False

        if self._producer_thread:
            self._producer_thread.join(timeout=2.0)
        if self._consumer_thread:
            self._consumer_thread.join(timeout=2.0)

        if self._sock:
            self._sock.close()
            self._sock = None

        logger.info(
            f"UDP受信サーバ停止: 受信={self.total_received}, "
            f"デコード={self.total_decoded}, エラー={self.total_errors}"
        )

    def _producer_loop(self):
        """
        Producer: ソケットからデータを受信し、Queue に投入する。
        できるだけ高速に動作し、デコード処理は行わない。
        """
        logger.info("Producerスレッド開始")
        while self._running:
            try:
                data, addr = self._sock.recvfrom(65535)
                recv_time = time.time()
                self.total_received += 1
                # 簡易バリデーション後に Queue へ
                if validate_packet_quick(data):
                    self.packet_queue.put((data, recv_time, addr))
                else:
                    self.total_errors += 1
                    logger.debug(
                        f"不正パケット破棄: addr={addr}, size={len(data)}"
                    )
            except socket.timeout:
                continue
            except OSError:
                if self._running:
                    logger.exception("ソケット受信エラー")
                break
        logger.info("Producerスレッド終了")

    def _consumer_loop(self):
        """
        Consumer: Queue からデータを取り出し、デコード・デバイス別振り分けを行う。
        """
        logger.info("Consumerスレッド開始")
        while self._running or not self.packet_queue.empty():
            try:
                raw, recv_time, addr = self.packet_queue.get(timeout=0.5)
            except Empty:
                continue

            try:
                pkt = decode_packet(raw, recv_timestamp=recv_time)
                self.total_decoded += 1

                # --- パケットロス検知 ---
                dev = pkt.device_id
                with self._lock:
                    prev_seq = self.last_seq[dev]
                    if prev_seq >= 0:
                        expected = prev_seq + 1
                        if pkt.sequence_no != expected:
                            lost = pkt.sequence_no - expected
                            if lost > 0:
                                self.lost_count[dev] += lost
                                logger.warning(
                                    f"パケットロス検知: Device {dev}, "
                                    f"期待={expected}, 受信={pkt.sequence_no}, "
                                    f"ロスト={lost}"
                                )
                    self.last_seq[dev] = pkt.sequence_no

                    # デバイス別データに振り分け
                    self.device_data[dev].append(pkt)

                # コールバック通知
                if self.on_packet_decoded:
                    try:
                        self.on_packet_decoded(pkt)
                    except Exception:
                        logger.exception("コールバック実行エラー")

            except PacketDecodeError as e:
                self.total_errors += 1
                logger.warning(f"パケットデコードエラー: {e} (from {addr})")

        logger.info("Consumerスレッド終了")

    def get_stats(self) -> dict:
        """現在の統計情報を返す"""
        with self._lock:
            stats = {
                "total_received": self.total_received,
                "total_decoded": self.total_decoded,
                "total_errors": self.total_errors,
                "queue_size": self.packet_queue.qsize(),
                "devices": {},
            }
            for dev_id in range(1, NUM_DEVICES + 1):
                stats["devices"][dev_id] = {
                    "packets": len(self.device_data[dev_id]),
                    "lost": self.lost_count[dev_id],
                    "last_seq": self.last_seq[dev_id],
                }
            return stats

    def clear_data(self):
        """蓄積データをクリアする"""
        with self._lock:
            for dev_id in range(1, NUM_DEVICES + 1):
                self.device_data[dev_id].clear()
                self.last_seq[dev_id] = -1
                self.lost_count[dev_id] = 0
            self.total_received = 0
            self.total_decoded = 0
            self.total_errors = 0
        logger.info("蓄積データをクリアしました")
