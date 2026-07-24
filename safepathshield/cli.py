from __future__ import annotations
import argparse,json
from pathlib import Path
from .core import DEMO,analyze,render_svg
from .server import serve
def main():
    p=argparse.ArgumentParser(prog="safepathshield"); s=p.add_subparsers(dest="command",required=True); run=s.add_parser("run");run.add_argument("input");render=s.add_parser("render");render.add_argument("output");s.add_parser("demo");server=s.add_parser("serve");server.add_argument("--host",default="127.0.0.1");server.add_argument("--port",type=int,default=8090);a=p.parse_args()
    if a.command=="serve":serve(a.host,a.port);return
    payload=DEMO if a.command in ("demo","render") else json.loads(Path(a.input).read_text());result=analyze(payload)
    if a.command=="render":result["artifact"]=str(render_svg(result,a.output))
    print(json.dumps(result,indent=2))
