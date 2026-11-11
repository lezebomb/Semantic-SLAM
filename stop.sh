#!/usr/bin/env bash
set -e
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
/usr/bin/python3 - "$root" <<'PY'
import json,os,pathlib,signal,sys,time
root=pathlib.Path(sys.argv[1]).resolve()
waiting=[]
for filename,script in [('active_live.json','run_live.py'),('active_run.json','run_demo.py')]:
 record=root/'logs'/filename
 if not record.exists():continue
 d=json.loads(record.read_text());pid=int(d['pid']);proc=pathlib.Path('/proc')/str(pid)
 try:
  actual=(proc/'stat').read_text().split()[21];cmd=(proc/'cmdline').read_bytes().split(b'\0')
 except FileNotFoundError:continue
 if actual!=d['start_ticks'] or str(root/'scripts'/script).encode() not in cmd:
  raise SystemExit('PID身份不匹配；没有操作该进程。')
 try:os.kill(pid,signal.SIGINT)
 except ProcessLookupError:continue
 waiting.append(d);print('已请求保存建图及录制；输出：'+d['output'],flush=True)
if not waiting:print('没有本组正在运行的托管任务；未操作其他进程。');raise SystemExit(0)
print('请停止推动相机或车体，等待保存完成。',flush=True)
until=time.monotonic()+120;notice=time.monotonic()
while waiting and time.monotonic()<until:
 remaining=[]
 for d in waiting:
  proc=pathlib.Path('/proc')/str(d['pid'])
  try:stat=(proc/'stat').read_text().split()
  except FileNotFoundError:continue
  if stat[21]==d['start_ticks'] and stat[2]!='Z':remaining.append(d)
 waiting=remaining
 if not waiting:break
 if time.monotonic()-notice>=5:print('正在等待本组保存退出……',flush=True);notice=time.monotonic()
 time.sleep(.2)
if waiting:raise SystemExit('保存退出尚未完成，请查看当前运行目录日志；没有强制终止其他进程。')
print('本组主管进程已退出；请查看对应 result.json 确认各项保存结果。')
PY
