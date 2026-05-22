#!/usr/bin/env python3
"""
camera_streamer.py — Ortak kamera stream modülü
Her görev kodu import eder.

Kullanım:
    from camera_streamer import CameraStreamer

    front  = CameraStreamer('/dev/video0', 5601, 'FRONT',  flip=True)
    bottom = CameraStreamer('/dev/video2', 5602, 'BOTTOM', flip=False)

    # Frame almak için:
    frame = front.get_frame()

    # Debug görüntüsü göndermek için:
    front.send_debug(vis_frame)

    # Kapatmak için:
    front.stop()
    bottom.stop()
"""

import cv2
import threading
import time
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

WIDTH   = 640
HEIGHT  = 480
FPS     = 15


class CameraStreamer:
    def __init__(self, device, port, name, flip=False, monster_ip='10.42.0.1'):
        self.name       = name
        self.flip       = flip
        self.running    = True
        self._frame     = None
        self._frame_lock = threading.Lock()
        self._debug_frame     = None
        self._debug_frame_lock = threading.Lock()

        # ── Kamera aç ────────────────────────────────────────
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if device == '/dev/video0':
            self.cap.set(cv2.CAP_PROP_FOURCC,
                         cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, FPS)

        if not self.cap.isOpened():
            print(f'[{name}] HATA: Kamera açılamadı ({device})')
            self.running = False
            return

        # ── GStreamer pipeline ────────────────────────────────
        Gst.init(None)
        pipeline_str = (
            f'appsrc name=src is-live=true block=true format=time '
            f'caps=video/x-raw,format=BGR,width={WIDTH},height={HEIGHT},'
            f'framerate={FPS}/1 ! '
            f'videoconvert ! video/x-raw,format=I420 ! '
            f'x264enc tune=zerolatency bitrate=800 speed-preset=ultrafast ! '
            f'rtph264pay ! '
            f'udpsink host={monster_ip} port={port} sync=false'
        )
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc   = self.pipeline.get_by_name('src')
        self.pipeline.set_state(Gst.State.PLAYING)

        # ── Thread'ler ────────────────────────────────────────
        threading.Thread(target=self._capture_loop, daemon=True).start()
        threading.Thread(target=self._stream_loop,  daemon=True).start()

        print(f'[{name}] Başladı → {device} → port {port}')

    # ── Capture loop: kameradan frame al, sakla ──────────────
    def _capture_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            if self.flip:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            with self._frame_lock:
                self._frame = frame

    # ── Stream loop: debug varsa onu, yoksa ham frame'i gönder
    def _stream_loop(self):
        interval = 1.0 / FPS
        while self.running:
            t0 = time.time()

            # Debug frame varsa onu gönder, yoksa ham
            with self._debug_frame_lock:
                frame = self._debug_frame
                self._debug_frame = None  # bir kez gönder

            if frame is None:
                with self._frame_lock:
                    frame = self._frame

            if frame is not None:
                self._push(frame)

            elapsed = time.time() - t0
            sleep   = interval - elapsed
            if sleep > 0:
                time.sleep(sleep)

    def _push(self, frame):
        try:
            # Boyutu garantile
            if frame.shape[1] != WIDTH or frame.shape[0] != HEIGHT:
                frame = cv2.resize(frame, (WIDTH, HEIGHT))
            data = frame.tobytes()
            buf  = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.duration = Gst.SECOND // FPS
            self.appsrc.emit('push-buffer', buf)
        except Exception as e:
            print(f'[{self.name}] Push hatası: {e}')

    # ── Public API ───────────────────────────────────────────

    def get_frame(self):
        """Son frame'i döndür (None olabilir)"""
        with self._frame_lock:
            return self._frame.copy() if self._frame is not None else None

    def send_debug(self, frame):
        """İşlenmiş debug görüntüsünü Yüzeye gönder"""
        with self._debug_frame_lock:
            self._debug_frame = frame.copy()

    def is_ready(self):
        """Kamera açık ve frame geliyor mu"""
        with self._frame_lock:
            return self._frame is not None

    def stop(self):
        self.running = False
        time.sleep(0.2)
        if self.cap.isOpened():
            self.cap.release()
        self.pipeline.set_state(Gst.State.NULL)
        print(f'[{self.name}] Durduruldu')