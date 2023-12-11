from opengl_renderer import OpenGLRenderer

from typing import Optional, Tuple

import os
import ctypes
import pygame
import win32gui
import win32ui
import win32con
import pygetwindow as gw
import numpy as np
import PIL
import time
from PIL import Image
from ctypes.wintypes import BOOL, POINT, RECT, BYTE, WORD, SIZE
 
# Constants and functions from dwmapi
DWM_TNP_VISIBLE = 0x8
DWM_TNP_RECTDESTINATION = 0x1
DWM_TNP_SOURCECLIENTAREAONLY = 0x10

class DWM_THUMBNAIL_PROPERTIES(ctypes.Structure):
    _fields_ = [
        ("dwFlags", WORD),
        ("rcDestination", RECT),
        ("rcSource", RECT),
        ("opacity", BYTE),
        ("fVisible", BOOL),
        ("fSourceClientAreaOnly", BOOL),
    ]

class OverlayDWM:
    SHADER = 'zoom_in'
    
    def __init__(self,
                 src_win_name: str,
                 src_win_point: Optional[Tuple[int, int]] = None) -> None:
        
        # Get capturing (source) window and its properties
        self._src_hwnd = self._get_source_window(src_win_name, src_win_point)
        self._src_win_rect = win32gui.GetWindowRect(self._src_hwnd)
        x = self._src_win_rect[0]
        y = self._src_win_rect[1]
        width = self._src_win_rect[2] - x
        height = self._src_win_rect[3] - y
        self._size = (width, height)
        self._location = x, y

        # Initialize GWM
        self._dwmapi = ctypes.windll.dwmapi
        self._thumbnail = ctypes.c_void_p()
        props = DWM_THUMBNAIL_PROPERTIES()
        props.dwFlags = DWM_TNP_VISIBLE | DWM_TNP_RECTDESTINATION | DWM_TNP_SOURCECLIENTAREAONLY
        props.fVisible = True
        props.fSourceClientAreaOnly = True
        self._dwmapi.DwmRegisterThumbnail(self._src_hwnd, ctypes.windll.user32.GetDesktopWindow(), ctypes.byref(self._thumbnail))
        self._dwmapi.DwmUpdateThumbnailProperties(self._thumbnail, ctypes.byref(props))

        # Create GL display (destination window)
        self._display = OpenGLRenderer(self._size, OverlayDWM.SHADER)

        icon_filename = os.path.join(os.getcwd(), 'scenario_runner', 'srunner', 'scenarios', 'emirror_overlay', 'images', 'icon.png')
        icon = pygame.image.load(icon_filename)
        pygame.display.set_icon(icon)
        
        dst_hwnd = pygame.display.get_wm_info()['window']
        win32gui.SetWindowPos(dst_hwnd, win32con.HWND_TOPMOST, x, y, 0, 0, win32con.SWP_NOSIZE)

    def run(self) -> None: 
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
            image = self._capture_window()
            self._render(image)
            
            pygame.display.flip()
            time.sleep(0.012)

        # Cleanup
        self._dwmapi.DwmUnregisterThumbnail(self._thumbnail)
        
    # Internal
    
    def _capture_window(self) -> Optional[Image.Image]:
        # Capture the thumbnail content
        img = None
        try:
            result = SIZE()
            self._dwmapi.DwmQueryThumbnailSourceSize(self._thumbnail, ctypes.byref(result))
            width = result.right - result.left
            height = result.bottom - result.top
            
            buffer = ctypes.create_string_buffer(width * height * 4)
            size = SIZE(width, height)
            self._dwmapi.DwmGetThumbnailBits(self._thumbnail, ctypes.byref(size), buffer)
            
            img = Image.frombytes("RGBA", (width, height), buffer, "raw", "BGRA", 0, 1)
        except Exception as e:
            print(f"Error capturing window content: {e}")
        
        return img
    
    def _render(self, image: Optional[Image.Image]) -> None:
        if image:
            # Convert captured image to a format usable by Pygame (if needed)
            image_data = image.convert("RGB")
            image_data = np.frombuffer(image_data.tobytes(), dtype=np.uint8).reshape((image.size[0], image.size[1], 3))
            self._display.render(image_data)
        
    def _get_source_window(self, window_title: str, point: Optional[Tuple[int,int]]) -> int:
        windows = gw.getWindowsWithTitle(window_title)
        if windows:
            if point is None:
                return windows[0]._hWnd
            else:
                for wnd in windows:
                    rect = win32gui.GetWindowRect(wnd._hWnd)
                    if win32gui.PtInRect(rect, point):
                        return wnd._hWnd
        
        raise Exception(f'Window "{window_title}" with point {point} not found.')
