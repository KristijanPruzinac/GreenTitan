"""Read-only validation and PrintWindow capture of the known shared Fusion window."""
import ctypes
import json
import runpy
import sys
from ctypes import wintypes
from pathlib import Path

u = ctypes.windll.user32
hwnd = 1247868
pid = wintypes.DWORD()
u.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
title = ctypes.create_unicode_buffer(1024)
u.GetWindowTextW(hwnd, title, len(title))
rect = wintypes.RECT()
u.GetClientRect(hwnd, ctypes.byref(rect))
state = {'hwnd': hwnd, 'pid': pid.value, 'title': title.value,
         'is_window': bool(u.IsWindow(hwnd)), 'minimized': bool(u.IsIconic(hwnd)),
         'client': [rect.right, rect.bottom]}
print(json.dumps(state))
if not state['is_window'] or pid.value != 23728 or state['minimized']:
    raise RuntimeError('Known Fusion window is not available; no input sent.')
if 'Fusion' not in state['title']:
    raise RuntimeError('Window title does not identify Fusion; no input sent.')
out = Path(__file__).resolve().parents[2] / 'outputs/spine_b/concept'
(out / 'fusion_window_probe.json').write_text(json.dumps(state, indent=2), encoding='utf-8')
helper = 'C:/Users/kikom/.codex/coordination/fusion/greentitan_window.py'
sys.argv = [helper, '--capture', str(out / 'fusion_before_import.png')]
runpy.run_path(helper, run_name='__main__')
