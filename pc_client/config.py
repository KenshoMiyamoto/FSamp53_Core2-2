"""
M5Stack UDP通信 - 設定ファイル
================================
通信パラメータ、パケット構造定数、保存設定をここで一元管理する。
"""

# ─── ネットワーク設定 ─────────────────────────────────────
# データ受信ポート (PC側)
DATA_PORT = 8000

# コマンド送信ポート (M5Stack側の待受ポート)
COMMAND_PORT = 8001

# ブロードキャストアドレス
BROADCAST_ADDR = "255.255.255.255"

# ─── パケット構造定数 ──────────────────────────────────────
# ヘッダ / フッタ マーカー
HEADER_MARKER = 0xAAAA
FOOTER_MARKER = 0x5555

# デバイス数 (M5Stack 1～6)
NUM_DEVICES = 6

# 1サンプルあたりのセンサチャンネル数 (float32 × 12ch)
NUM_CHANNELS = 12

# 1パケットあたりのサンプル数 (パッキング数)
SAMPLES_PER_PACKET = 10

# サンプリングレート [Hz] (100～200)
SAMPLING_RATE = 200

# ─── バイトサイズ定数 ──────────────────────────────────────
HEADER_SIZE = 2       # uint16
DEVICE_ID_SIZE = 1    # uint8
SEQ_NO_SIZE = 4       # uint32
TIMESTAMP_SIZE = 4    # uint32 (マイクロ秒)
SENSOR_DATA_SIZE = NUM_CHANNELS * 4  # float32 × 12 = 48 bytes
SAMPLE_SIZE = TIMESTAMP_SIZE + SENSOR_DATA_SIZE  # 52 bytes
FOOTER_SIZE = 2       # uint16

# パケット全体サイズ
PACKET_SIZE = (
    HEADER_SIZE
    + DEVICE_ID_SIZE
    + SEQ_NO_SIZE
    + SAMPLES_PER_PACKET * SAMPLE_SIZE
    + FOOTER_SIZE
)

# ─── バッファ設定 ──────────────────────────────────────────
# OS UDP受信バッファサイズ (1 MB 以上)
UDP_RECV_BUFFER_SIZE = 4 * 1024 * 1024  # 4 MB

# Python側 Queue の最大サイズ (0 = 無制限)
QUEUE_MAX_SIZE = 0

# ─── コマンド定義 ──────────────────────────────────────────
CMD_START = b"START"
CMD_STOP = b"STOP"

# ─── 保存設定 ──────────────────────────────────────────────
# CSVの出力ディレクトリ
OUTPUT_DIR = "data"

# チャンネル名定義
CHANNEL_NAMES = [
    "sensor_1", "sensor_2", "sensor_3",      # CH 1-3: センサ値
    "accel_x",  "accel_y",  "accel_z",       # CH 4-6: 加速度
    "gyro_x",   "gyro_y",   "gyro_z",        # CH 7-9: 角速度
    "load_1",   "load_2",   "load_3",        # CH 10-12: 荷重
]
