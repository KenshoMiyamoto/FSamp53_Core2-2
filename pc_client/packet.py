"""
M5Stack UDP通信 - パケットデコーダ
===================================
受信したバイナリパケットを解析し、構造化データに変換する。

パケット構造:
  Header     (2 Bytes) : 0xAAAA
  Device ID  (1 Byte)  : 1 ～ 6
  Seq No.    (4 Bytes) : uint32 連番
  Payload × N:
    Timestamp   (4 Bytes)  : uint32 マイクロ秒
    Sensor Data (48 Bytes) : float32 × 12ch
  Footer     (2 Bytes) : 0x5555
"""

import struct
from dataclasses import dataclass, field
from typing import Optional

from config import (
    HEADER_MARKER,
    FOOTER_MARKER,
    NUM_CHANNELS,
    SAMPLES_PER_PACKET,
    PACKET_SIZE,
    HEADER_SIZE,
    DEVICE_ID_SIZE,
    SEQ_NO_SIZE,
    SAMPLE_SIZE,
    TIMESTAMP_SIZE,
    SENSOR_DATA_SIZE,
    FOOTER_SIZE,
)


@dataclass
class SampleData:
    """1サンプル分のデータ"""
    timestamp_us: int              # マイクロ秒単位のタイムスタンプ
    channels: list[float] = field(default_factory=list)  # 12ch センサデータ


@dataclass
class DecodedPacket:
    """デコード済みパケット"""
    device_id: int                         # デバイス識別番号 (1-6)
    sequence_no: int                       # シーケンス番号
    samples: list[SampleData] = field(default_factory=list)  # サンプルデータのリスト
    recv_timestamp: float = 0.0            # PC側受信時刻 (time.time())


class PacketDecodeError(Exception):
    """パケットデコード失敗時の例外"""
    pass


def decode_packet(raw: bytes, recv_timestamp: float = 0.0) -> DecodedPacket:
    """
    バイナリパケットをデコードする。

    Parameters
    ----------
    raw : bytes
        受信した生パケットデータ
    recv_timestamp : float
        PC側での受信時刻 (time.time())

    Returns
    -------
    DecodedPacket
        デコード済みのパケットデータ

    Raises
    ------
    PacketDecodeError
        パケットが不正な場合
    """
    # --- サイズチェック ---
    if len(raw) != PACKET_SIZE:
        raise PacketDecodeError(
            f"パケットサイズ不正: 期待={PACKET_SIZE}, 実際={len(raw)}"
        )

    offset = 0

    # --- ヘッダ検証 ---
    header = struct.unpack_from(">H", raw, offset)[0]
    if header != HEADER_MARKER:
        raise PacketDecodeError(
            f"ヘッダ不正: 期待=0x{HEADER_MARKER:04X}, 実際=0x{header:04X}"
        )
    offset += HEADER_SIZE

    # --- Device ID ---
    device_id = struct.unpack_from("B", raw, offset)[0]
    if not (1 <= device_id <= 6):
        raise PacketDecodeError(
            f"Device ID 範囲外: {device_id} (1-6 を期待)"
        )
    offset += DEVICE_ID_SIZE

    # --- Sequence No. ---
    sequence_no = struct.unpack_from(">I", raw, offset)[0]
    offset += SEQ_NO_SIZE

    # --- Payload (N samples) ---
    samples: list[SampleData] = []
    for _ in range(SAMPLES_PER_PACKET):
        # Timestamp (uint32, マイクロ秒)
        timestamp_us = struct.unpack_from(">I", raw, offset)[0]
        offset += TIMESTAMP_SIZE

        # Sensor Data (float32 × 12ch)
        fmt = f">{NUM_CHANNELS}f"
        channels = list(struct.unpack_from(fmt, raw, offset))
        offset += SENSOR_DATA_SIZE

        samples.append(SampleData(timestamp_us=timestamp_us, channels=channels))

    # --- フッタ検証 ---
    footer = struct.unpack_from(">H", raw, offset)[0]
    if footer != FOOTER_MARKER:
        raise PacketDecodeError(
            f"フッタ不正: 期待=0x{FOOTER_MARKER:04X}, 実際=0x{footer:04X}"
        )

    return DecodedPacket(
        device_id=device_id,
        sequence_no=sequence_no,
        samples=samples,
        recv_timestamp=recv_timestamp,
    )


def validate_packet_quick(raw: bytes) -> bool:
    """
    パケットの簡易バリデーション（ヘッダ/フッタ/サイズのみ）。
    フルデコード前のフィルタリングに使用。

    Parameters
    ----------
    raw : bytes
        受信した生パケットデータ

    Returns
    -------
    bool
        パケットが有効そうであれば True
    """
    if len(raw) != PACKET_SIZE:
        return False
    header = struct.unpack_from(">H", raw, 0)[0]
    footer = struct.unpack_from(">H", raw, PACKET_SIZE - FOOTER_SIZE)[0]
    return header == HEADER_MARKER and footer == FOOTER_MARKER
