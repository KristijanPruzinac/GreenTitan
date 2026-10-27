"""Pass a UTF-8 command file to the window helper without native-shell quoting."""
import argparse,runpy,sys
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('file');args=p.parse_args()
command=Path(args.file).read_text(encoding='utf-8').strip()
if '\n' in command:raise RuntimeError('Console loader must be one line')
helper='C:/Users/kikom/.codex/coordination/fusion/greentitan_window.py'
sys.argv=[helper,'--command',command]
runpy.run_path(helper,run_name='__main__')
