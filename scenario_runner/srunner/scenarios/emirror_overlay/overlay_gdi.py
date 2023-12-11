from opengl_renderer import OpenGLRenderer

from typing import Optional, Tuple

import os
import pygame
import win32gui
import win32ui
import win32con
import pygetwindow as gw
import numpy as np
import ctypes
from PIL import Image
from time import sleep

class OverlayGDI:
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

        # Initialize a GDI buffer for copying window's content
        self._hwnd_dc = win32gui.GetWindowDC(self._src_hwnd)
        self._src_dc = win32ui.CreateDCFromHandle(self._hwnd_dc) 
        self._dst_dc = self._src_dc.CreateCompatibleDC()                                             

        self._bmp = win32ui.CreateBitmap()    
        self._bmp.CreateCompatibleBitmap(self._src_dc, width, height)    
        self._dst_dc.SelectObject(self._bmp)      
        
        # Create GL display (destination window)
        self._display = OpenGLRenderer(self._size, OverlayGDI.SHADER)

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
            sleep(0.012)

        # Cleanup
        win32gui.DeleteObject(self._bmp.GetHandle())
        self._src_dc.DeleteDC()
        self._dst_dc.DeleteDC()
        win32gui.ReleaseDC(self._src_hwnd, self._hwnd_dc)
        
    # Internal
    
    def _capture_window(self) -> Image.Image:

        # BitBlt does not capture windows with hardware acceleration enabled
        # self._dst_dc.BitBlt((0, 0), self._size, self._src_dc, (0, 0), win32con.SRCCOPY) 

        # PrintWindow should capture windows with hardware acceleration enabled
        PW_RENDERFULLCONTENT = 0x02
        ctypes.windll.user32.PrintWindow(self._src_hwnd, self._dst_dc.GetSafeHdc(), PW_RENDERFULLCONTENT)

        bmp_array = self._bmp.GetBitmapBits(True)
        return Image.frombuffer("RGB", self._size, bmp_array, "raw", "BGRX", 0, 1)

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
