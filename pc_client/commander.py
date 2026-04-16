"""
M5Stack UDP通信 - コマンド送信モジュール
=========================================
ブロードキャスト経由で START / STOP コマンドを M5Stack に送信する。
"""

import socket
import logging

from config import COMMAND_PORT, BROADCAST_ADDR, CMD_START, CMD_STOP

logger = logging.getLogger(__name__)


class CommandSender:
    """
    M5Stack へのコマンド送信を管理するクラス。
    ブロードキャストアドレス宛に UDP でコマンドを送信する。
    """

    def __init__(
        self,
        command_port: int = COMMAND_PORT,
        broadcast_addr: str = BROADCAST_ADDR,
    ):
        self.command_port = command_port
        self.broadcast_addr = broadcast_addr
        self._sock: socket.socket | None = None

    def _get_socket(self) -> socket.socket:
        """ブロードキャスト対応ソケットを取得する"""
        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        return self._sock

    def _send_command(self, cmd: bytes, label: str):
        """コマンドを送信する (ブロードキャスト失敗時は localhost にフォールバック)"""
        sock = self._get_socket()
        try:
            sock.sendto(cmd, (self.broadcast_addr, self.command_port))
            logger.info(
                f"{label} コマンド送信: {self.broadcast_addr}:{self.command_port}"
            )
        except OSError as e:
            # ブロードキャスト不可の場合 (ネットワーク未接続など)、localhost へフォールバック
            logger.warning(
                f"ブロードキャスト送信失敗 ({e}), localhost にフォールバック"
            )
            sock.sendto(cmd, ("127.0.0.1", self.command_port))
            logger.info(
                f"{label} コマンド送信 (fallback): 127.0.0.1:{self.command_port}"
            )

    def send_start(self):
        """
        START コマンドを全M5Stackにブロードキャスト送信する。
        M5Stackはこのコマンドを受けて内部タイマーを0にリセットし、計測を開始する。
        """
        self._send_command(CMD_START, "START")

    def send_stop(self):
        """
        STOP コマンドを全M5Stackにブロードキャスト送信する。
        M5Stackは計測と送信を停止し、待機フェーズに戻る。
        """
        self._send_command(CMD_STOP, "STOP")

    def close(self):
        """ソケットを閉じる"""
        if self._sock:
            self._sock.close()
            self._sock = None
