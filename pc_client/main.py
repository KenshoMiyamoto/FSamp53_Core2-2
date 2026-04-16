"""
M5Stack UDP通信 - メインアプリケーション
==========================================
CLI インターフェースで計測の開始・停止・保存を制御する。

使い方:
    python main.py

コマンド:
    start  : 計測を開始 (STARTコマンド送信 + 受信開始)
    stop   : 計測を停止 (STOPコマンド送信 + 受信停止)
    save   : 蓄積データをCSVに保存
    status : 統計情報を表示
    quit   : プログラムを終了
"""

import sys
import time
import logging
import signal

from config import DATA_PORT, NUM_DEVICES
from receiver import UDPReceiver
from commander import CommandSender
from saver import DataSaver
from packet import DecodedPacket

# ─── ロギング設定 ──────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("main")


class MeasurementApp:
    """計測アプリケーション本体"""

    def __init__(self):
        self.receiver = UDPReceiver(
            port=DATA_PORT,
            on_packet_decoded=self._on_packet,
        )
        self.commander = CommandSender()
        self.saver = DataSaver()
        self.measurement_start_time: float | None = None
        self._is_measuring = False

    def _on_packet(self, pkt: DecodedPacket):
        """パケットデコード完了時のコールバック (リアルタイムログ用)"""
        # 大量のログが出るためデバッグ時のみ有効化
        logger.debug(
            f"Device {pkt.device_id} | Seq {pkt.sequence_no} | "
            f"Samples: {len(pkt.samples)}"
        )

    def start_measurement(self):
        """計測を開始する"""
        if self._is_measuring:
            print("⚠ 既に計測中です")
            return

        # 受信サーバを先に起動
        self.receiver.clear_data()
        self.receiver.start()

        # STARTコマンド送信 (計測開始時刻を記録)
        self.measurement_start_time = time.time()
        self.commander.send_start()

        self._is_measuring = True
        print("✅ 計測を開始しました")
        print(
            f"   開始時刻: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.measurement_start_time))}"
        )
        print(f"   受信ポート: {DATA_PORT}")

    def stop_measurement(self):
        """計測を停止する"""
        if not self._is_measuring:
            print("⚠ 計測は開始されていません")
            return

        # STOPコマンド送信
        self.commander.send_stop()

        # 少し待って残留パケットを受信
        print("   残留パケット受信待機中...")
        time.sleep(1.0)

        # 受信サーバ停止
        self.receiver.stop()
        self._is_measuring = False

        print("✅ 計測を停止しました")
        self._print_stats()

    def save_data(self):
        """蓄積データをCSVに保存する"""
        if self._is_measuring:
            print("⚠ 計測中です。先に stop してください")
            return

        has_data = any(
            len(self.receiver.device_data[d]) > 0
            for d in range(1, NUM_DEVICES + 1)
        )
        if not has_data:
            print("⚠ 保存するデータがありません")
            return

        print("💾 データを保存中...")
        saved = self.saver.save_all_devices(
            self.receiver.device_data,
            measurement_start_time=self.measurement_start_time,
        )
        if saved:
            print(f"✅ {len(saved)} ファイルを保存しました:")
            for f in saved:
                print(f"   📄 {f}")
        else:
            print("⚠ 保存するデータがありませんでした")

    def _print_stats(self):
        """統計情報を表示する"""
        stats = self.receiver.get_stats()
        print("\n" + "=" * 60)
        print("📊 統計情報")
        print("=" * 60)
        print(f"  受信パケット数  : {stats['total_received']}")
        print(f"  デコード成功    : {stats['total_decoded']}")
        print(f"  エラー          : {stats['total_errors']}")
        print(f"  キュー残量      : {stats['queue_size']}")
        print("-" * 60)
        print(f"  {'Device':>8} | {'Packets':>10} | {'Lost':>8} | {'LastSeq':>10}")
        print(f"  {'':->8}-+-{'':->10}-+-{'':->8}-+-{'':->10}")
        for dev_id, info in stats["devices"].items():
            print(
                f"  {dev_id:>8} | {info['packets']:>10} | "
                f"{info['lost']:>8} | {info['last_seq']:>10}"
            )
        print("=" * 60)

    def run(self):
        """メインループ (CLI)"""
        print()
        print("=" * 60)
        print("  M5Stack UDP 受信クライアント")
        print("=" * 60)
        print()
        print("コマンド:")
        print("  start  - 計測開始 (START送信 + 受信開始)")
        print("  stop   - 計測停止 (STOP送信 + 受信停止)")
        print("  save   - データをCSVに保存")
        print("  status - 統計情報を表示")
        print("  quit   - プログラム終了")
        print()

        # Ctrl+C ハンドラ
        def signal_handler(sig, frame):
            print("\n⏹ 中断されました")
            if self._is_measuring:
                self.stop_measurement()
            self.commander.close()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

        while True:
            try:
                cmd = input(">>> ").strip().lower()
            except EOFError:
                break

            if cmd == "start":
                self.start_measurement()
            elif cmd == "stop":
                self.stop_measurement()
            elif cmd == "save":
                self.save_data()
            elif cmd == "status":
                self._print_stats()
            elif cmd in ("quit", "exit", "q"):
                if self._is_measuring:
                    self.stop_measurement()
                self.commander.close()
                print("👋 終了します")
                break
            elif cmd == "":
                continue
            else:
                print(f"⚠ 不明なコマンド: {cmd}")


if __name__ == "__main__":
    app = MeasurementApp()
    app.run()
