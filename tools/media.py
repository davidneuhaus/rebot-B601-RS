#!/usr/bin/env python3
"""Medien-Helfer (ffmpeg).
  media.py image  <quelle> [name]                 -> site/media/<name>.webp (max 1600px)
  media.py video  <quelle> <start> <dauer> [name] [--audio] [--width 1280]
                                                  -> site/media/<name>.mp4 (<5MB) + <name>.jpg (Poster)
Start z.B. 12.5 oder 00:01:05; Dauer in Sekunden."""
import sys, subprocess, re
from pathlib import Path
OUT = Path(__file__).resolve().parent.parent/"site"/"media"
def run(c): subprocess.run(c, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
def slug(p): return re.sub(r"[^a-z0-9]+","-",Path(p).stem.lower()).strip("-")
def image(src, name=None):
    name = name or slug(src); OUT.mkdir(parents=True, exist_ok=True)
    o = OUT/f"{name}.webp"
    run(["ffmpeg","-y","-i",src,"-vf","scale='min(1600,iw)':-2","-q:v","80","-frames:v","1",str(o)])
    print(o, o.stat().st_size//1024, "KB"); print(f"Markdown: ![Bildunterschrift]({o.name})")
def video(src, start, dur, name=None, audio=False, width=1280):
    name = name or slug(src); OUT.mkdir(parents=True, exist_ok=True)
    o = OUT/f"{name}.mp4"; budget = 4.6e6*8  # bits
    br = int(min(2_500_000, budget/float(dur)*0.9))
    for _ in range(4):
        c = ["ffmpeg","-y","-ss",str(start),"-t",str(dur),"-i",src,"-vf",f"scale='min({width},iw)':-2,fps=30",
             "-c:v","libx264","-profile:v","main","-pix_fmt","yuv420p","-b:v",str(br),"-maxrate",str(int(br*1.3)),"-bufsize",str(br*2),"-movflags","+faststart"]
        c += ["-c:a","aac","-b:a","96k"] if audio else ["-an"]
        run(c+[str(o)])
        if o.stat().st_size < 5_000_000: break
        br = int(br*0.7)
    run(["ffmpeg","-y","-ss",str(start),"-i",src,"-frames:v","1","-vf",f"scale='min({width},iw)':-2","-q:v","4",str(OUT/f"{name}.jpg")])
    print(o, round(o.stat().st_size/1e6,2), "MB"); print(f"Markdown: ![Bildunterschrift]({o.name})")
if __name__=="__main__":
    a=[x for x in sys.argv[1:] if not x.startswith("--")]; audio="--audio" in sys.argv
    w=int(sys.argv[sys.argv.index("--width")+1]) if "--width" in sys.argv else 1280
    if "--width" in sys.argv: a.remove(str(w))
    if len(a)>=2 and a[0]=="image": image(a[1], *(a[2:3]))
    elif len(a)>=4 and a[0]=="video": video(a[1],a[2],a[3],*(a[4:5]),audio=audio,width=w)
    else: print(__doc__); sys.exit(1)
