# models/dm_serial.py
# -*- coding: utf-8 -*-
"""
DM_Serial: 达妙 IMU 串口读取类（支持“后台读线程” + 主线程按需取最新）
- 固定 timeout=0（非阻塞）
- read(): Drain+ParseAll+Latest（也可单线程调用）
- start_reader()/stop_reader(): 在内部起一个只负责“刷新数据”的线程，不打印
- get_latest(): 主线程随时取“最新一帧”与其时间戳/计数
- destory()/reopen(): 资源管理
- CRC：默认“包含帧头 0x55,0xAA”，失败自动再试“不含帧头”

帧格式：
[0,1]=0x55,0xAA | [2]=? | [3]=RID | [4:16]=3*float32(LE) | [16:18]=CRC16(LE) | [18]=0x0A
"""
from __future__ import annotations

import struct
import threading
import time
from typing import Optional, Tuple, List

import serial  # pip install pyserial

from .dm_crc import dm_crc16

HDR = b'\x55\xAA'
TAIL = 0x0A
FRAME_LEN = 19
VALID_RIDS = {0x01, 0x02, 0x03}

# 你的固件：CRC 计算应包含帧头，如需兼容其它版本可切换；我们也做了兜底。
SKIP_HDR_IN_CRC = False

class DM_Serial:
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout = 0.0  # 非阻塞
        self.ser: Optional[serial.Serial] = None
        self._buf = bytearray()

        # 统计
        self.cnt_ok = 0
        self.cnt_crc = 0
        self.cnt_short = 0
        self.cnt_nohdr = 0

        # 后台读线程状态
        self._th: Optional[threading.Thread] = None
        self._stop_evt: Optional[threading.Event] = None
        self._read_sleep = 0.001  # 读线程小睡控制 CPU

        # 最新数据（线程安全）
        self._latest_lock = threading.Lock()
        self._latest_pkt: Optional[Tuple[int, Tuple[float, float, float]]] = None
        self._latest_ts: float = 0.0
        self._latest_count: int = 0
        self._last_error: Optional[str] = None

        self._open()

    # ------------ 公共 API ------------
    def read(self, max_bytes: int | None = None) -> Optional[Tuple[int, Tuple[float, float, float]]]:
        """一次性读入串口当前可读字节，解析所有完整帧，只返回“最新一帧”。"""
        if not self.ser or not self.ser.is_open:
            return None
        self._read_into_buf(max_bytes)
        frames = self._parse_all()
        return frames[-1] if frames else None

    def start_reader(self, read_sleep: float = 0.001) -> bool:
        """启动只负责刷新数据的后台线程；不打印。"""
        if self._th and self._th.is_alive():
            self._read_sleep = read_sleep
            return True
        if not self.is_open:
            if not self._open():
                return False
        self._stop_evt = threading.Event()
        self._read_sleep = read_sleep
        self._th = threading.Thread(target=self._reader_loop, daemon=True)
        self._th.start()
        return True

    def stop_reader(self) -> None:
        """停止后台读线程。"""
        if self._stop_evt:
            self._stop_evt.set()
        if self._th:
            self._th.join(timeout=1.0)
        self._th = None
        self._stop_evt = None

    def get_latest(self) -> Tuple[Optional[Tuple[int, Tuple[float, float, float]]], float, int]:
        """线程安全地获取（pkt, timestamp, count）。"""
        with self._latest_lock:
            return self._latest_pkt, self._latest_ts, self._latest_count

    def last_error(self) -> Optional[str]:
        return self._last_error

    def destory(self) -> None:
        """立即关闭串口（按你的拼写保留）。"""
        self.stop_reader()
        if self.ser:
            try:
                self.ser.close()
            finally:
                self.ser = None

    # 别名
    def destroy(self) -> None:
        self.destory()

    def reopen(self) -> bool:
        """关闭并重新打开串口。"""
        self.destory()
        return self._open()

    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    # ------------ 内部实现 ------------
    def _open(self) -> bool:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout, write_timeout=0)
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            return True
        except Exception as e:
            self._last_error = str(e)
            self.ser = None
            return False

    def _reader_loop(self):
        """后台线程：不断刷新最新数据，不打印。"""
        evt = self._stop_evt
        try:
            while evt and not evt.is_set():
                pkt = self.read(None)
                if pkt is not None:
                    with self._latest_lock:
                        self._latest_pkt = pkt
                        self._latest_ts = time.time()
                        self._latest_count += 1
                if self._read_sleep > 0.0:
                    time.sleep(self._read_sleep)
        except Exception as e:
            # 不打印，记录错误字符串，便于主线程查询
            self._last_error = f"reader_loop: {e!r}"

    def _read_into_buf(self, max_bytes: Optional[int]) -> int:
        """把串口里“当前可读”的字节读入缓冲；返回读取字节数。"""
        n = getattr(self.ser, "in_waiting", 0) if self.ser else 0
        if max_bytes is not None and n > max_bytes:
            n = max_bytes
        if n <= 0:
            return 0
        self._buf.extend(self.ser.read(n))
        return n

    def _parse_all(self) -> List[Tuple[int, Tuple[float, float, float]]]:
        """解析尽可能多的完整帧，返回列表。"""
        results: List[Tuple[int, Tuple[float, float, float]]] = []
        buf = self._buf
        start = 0

        while True:
            j = buf.find(HDR, start)
            if j < 0:
                # 只保留最后 1 个字节，避免帧头跨包
                keep = buf[-1:] if buf else b''
                self._buf = bytearray(keep)
                if buf:
                    self.cnt_nohdr += 1
                break

            if len(buf) - j < FRAME_LEN:
                # 不够一帧，保留从帧头开始的尾部
                self._buf = bytearray(buf[j:])
                self.cnt_short += 1
                break

            frame = bytes(buf[j:j + FRAME_LEN])
            start = j + 1  # 快速向前移动

            # 尾字节检查
            if frame[-1] != TAIL:
                continue

            rid = frame[3]
            if rid not in VALID_RIDS:
                continue

            # CRC（默认含帧头，失败再试不含帧头）
            if SKIP_HDR_IN_CRC:
                crc_calc = dm_crc16(frame[2:16])
            else:
                crc_calc = dm_crc16(frame[0:16])
            crc_wire = frame[16] | (frame[17] << 8)
            if crc_calc != crc_wire:
                alt = dm_crc16(frame[2:16]) if not SKIP_HDR_IN_CRC else dm_crc16(frame[0:16])
                if alt != crc_wire:
                    self.cnt_crc += 1
                    continue

            # 解 3 个 float32（LE）
            f1 = struct.unpack('<f', frame[4:8])[0]
            f2 = struct.unpack('<f', frame[8:12])[0]
            f3 = struct.unpack('<f', frame[12:16])[0]
            results.append((rid, (f1, f2, f3)))

            # 丢弃已消费的数据（到帧尾），并从头继续找
            buf = buf[j + FRAME_LEN:]
            start = 0

        if isinstance(buf, (bytes, bytearray)) and buf is not self._buf:
            self._buf = bytearray(buf)

        self.cnt_ok += len(results)
        return results
