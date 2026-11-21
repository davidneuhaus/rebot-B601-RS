"""GUI test for the v3.7 enlarge window (needs a display; a test window opens for ~3 s). Feeds are simulated.
Usage: ~/humanoid/.venv/bin/python test_enlarge_v37.py [~/humanoid]"""
import sys, time, numpy as np, tkinter as tk
import os; sys.path.insert(0, os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else "~/humanoid"))
import camera_feeds as c
fails = []
def check(n, ok):
    print(("OK   " if ok else "FAIL ") + n)
    if not ok: fails.append(n)
root = tk.Tk(); root.title("test_enlarge_v37")
f = c.Feed("Test: Übersicht", "name:C920", (360, 240), 15)
fr, upd = c.make_panel(root, f, (360, 240)); fr.pack()
def feed():
    f._got_frame(bgr=(np.random.rand(1080, 1920, 3) * 255).astype(np.uint8)); root.after(33, feed)
feed()
for _ in range(20): upd(); root.update(); time.sleep(0.03)
btn = [w for w in fr.winfo_children()[1].winfo_children() if isinstance(w, tk.Button)][0]
check("panel has the ⤢ button", btn.cget("text") == "⤢")
btn.invoke(); root.update()
top = c._BIG[f.name]; top.geometry("1600x900")
t0 = time.time(); n = 0
while time.time() - t0 < 1.5: root.update(); time.sleep(0.01)
cv = [w for w in top.winfo_children() if isinstance(w, tk.Canvas)][0]
img = cv.find_all()[0]; ph = cv.itemcget(img, "image")
pw = int(root.tk.call("image", "width", ph)) if ph else 0; phh = int(root.tk.call("image", "height", ph)) if ph else 0
check(f"enlarge window shows the frame fitted to the canvas {cv.winfo_width()}x{cv.winfo_height()}: image {pw}x{phh}",
      pw > 0 and abs(pw / max(1, phh) - 16 / 9) < 0.02 and (pw == cv.winfo_width() or phh == cv.winfo_height()))
c.open_large(root, f); check("second ⤢ reuses the open window", len(c._BIG) == 1)
top.event_generate("<Escape>"); root.update()
check("Esc closes", f.name not in c._BIG)
s0 = f._seq
for _ in range(10): upd(); root.update(); time.sleep(0.03)
check("feed keeps running after close", f._seq > s0 and f.has_signal())
canv = fr.winfo_children()[0]
check("double-click is bound on the feed picture", bool(canv.bind("<Double-Button-1>")))
root.tk.eval(canv.bind("<Double-Button-1>").replace("%W", str(canv)).replace("%x", "10").replace("%y", "10").replace("%#", "0")
             .replace("%b", "1").replace("%t", "0").replace("%K", "??").replace("%k", "0").replace("%A", "{}").replace("%T", "4")
             .replace("%X", "0").replace("%Y", "0").replace("%D", "0").replace("%E", "0").replace("%N", "0").replace("%R", "0")
             .replace("%S", "0").replace("%s", "0").replace("%d", "0").replace("%h", "0").replace("%w", "0").replace("%B", "0")
             .replace("%c", "0").replace("%f", "0").replace("%m", "0").replace("%o", "0").replace("%p", "0").replace("%a", "0")
             .replace("%i", "0").replace("%P", "0").replace("%M", "0")); root.update()
check("double-click handler opens the window", f.name in c._BIG)
xb = [w for w in c._BIG[f.name].winfo_children()[0].winfo_children() if isinstance(w, tk.Button)][0]
check("window has a ✕ button", xb.cget("text") == "✕")
xb.invoke(); root.update()
check("✕ closes", f.name not in c._BIG)
root.destroy()
print(f"\n{len(fails)} Fehler"); sys.exit(1 if fails else 0)
