"""iPhone browser camera bridge (2 channels).

HTTPS (self-signed) page on :8444 uses getUserMedia on the iPhone and pushes JPEG frames over a
WebSocket. Each phone picks a channel with the URL: https://<LAN-IP>:8444/?cam=1 or ?cam=2
(no parameter = cam 1). Frames are re-served for OpenCV/LeRobot/the teleop GUI on 127.0.0.1:8090:
  /stream1.mjpg, /stream2.mjpg   MJPEG per channel (/stream.mjpg = alias for cam 1)
  /latest1.jpg,  /latest2.jpg    newest JPEG per channel (/latest.jpg, /snapshot.jpg = cam 1)
  /status                        JSON for both channels (/status1, /status2 per channel)
Security: the HTTPS page, its /status and the WebSocket require ?t=<token> (token.txt, chmod 600,
generated on first start); without a valid token -> 403. The MJPEG side listens on 127.0.0.1 only.
The full phone links are written to links.txt (chmod 600) by start.sh / on server start.
"""
import asyncio, hmac, json, os, secrets, socket, ssl, struct, subprocess, time
from pathlib import Path
from aiohttp import web, WSMsgType

HERE = Path(__file__).resolve().parent
HTTPS_PORT = int(os.environ.get("IPHONE_WEB_CAM_HTTPS_PORT", "8444"))
MJPEG_PORT = int(os.environ.get("IPHONE_WEB_CAM_MJPEG_PORT", "8090"))
CAMS = (1, 2)
TMP = Path("/tmp/iphone_web_cam"); TMP.mkdir(parents=True, exist_ok=True)
TOKEN_FILE = HERE / "token.txt"
LINKS_FILE = HERE / "links.txt"


def load_token() -> str:
    """Read token.txt or create it once (random, URL-safe, chmod 600)."""
    if TOKEN_FILE.exists():
        t = TOKEN_FILE.read_text().strip()
        if len(t) >= 16:
            os.chmod(TOKEN_FILE, 0o600)
            return t
    t = secrets.token_urlsafe(24)
    fd = os.open(TOKEN_FILE, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(t + "\n")
    os.chmod(TOKEN_FILE, 0o600)
    return t


def lan_ip() -> str:
    for iface in ("en0", "en1"):
        try:
            ip = subprocess.run(["ipconfig", "getifaddr", iface], capture_output=True, text=True, timeout=2).stdout.strip()
            if ip:
                return ip
        except Exception:
            pass
    return "<LAN-IP>"


def write_links(token: str):
    ip = lan_ip()
    txt = "".join(f"iPhone {c}: https://{ip}:{HTTPS_PORT}/?cam={c}&t={token}\n" for c in CAMS)
    fd = os.open(LINKS_FILE, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(txt)
    os.chmod(LINKS_FILE, 0o600)


TOKEN = load_token()


def token_ok(req) -> bool:
    return hmac.compare_digest(req.query.get("t", ""), TOKEN)


@web.middleware
async def require_token(req, handler):
    if not token_ok(req):
        print(f"[auth] 403 {req.remote} {req.path}", flush=True)
        return web.Response(status=403, text="403 forbidden: missing or wrong token\n")
    return await handler(req)


class State:
    def __init__(self, cam):
        self.cam = cam
        self.jpeg = None; self.count = 0; self.w = self.h = 0; self.fps = 0.0; self.last_ts = 0.0
        self._t0 = time.time(); self._c0 = 0
        self.cond = None; self.last_file = 0.0; self.clients = 0
        # cam 1 keeps the old file name so existing scripts keep working
        self.file = TMP / ("latest.jpg" if cam == 1 else f"latest{cam}.jpg")


S = {c: State(c) for c in CAMS}


def jpeg_size(b: bytes):
    i = 2
    while i < len(b) - 9:
        if b[i] != 0xFF:
            i += 1; continue
        m = b[i + 1]
        if m in (0xC0, 0xC1, 0xC2):
            h, w = struct.unpack(">HH", b[i + 5:i + 9]); return w, h
        i += 2 + struct.unpack(">H", b[i + 2:i + 4])[0]
    return 0, 0


def cam_of(req, key="cam"):
    try:
        c = int(req.query.get(key, "1"))
    except ValueError:
        c = 1
    return c if c in S else 1


async def push_frame(st: State, b: bytes):
    st.jpeg = b; st.count += 1; st.last_ts = time.time()
    if st.count == 1 or st.count % 30 == 0:
        st.w, st.h = jpeg_size(b)
    dt = st.last_ts - st._t0
    if dt >= 2:
        st.fps = (st.count - st._c0) / dt; st._t0 = st.last_ts; st._c0 = st.count
    if st.last_ts - st.last_file > 0.2:
        st.last_file = st.last_ts
        tmp = st.file.with_suffix(".tmp")
        tmp.write_bytes(b); os.replace(tmp, st.file)
    async with st.cond:
        st.cond.notify_all()


# ---------------- HTTPS side (iPhone) ----------------
async def index(req):
    return web.FileResponse(HERE / "index.html")


async def ws_handler(req):
    st = S[cam_of(req)]
    ws = web.WebSocketResponse(max_msg_size=8 * 1024 * 1024)
    await ws.prepare(req)
    st.clients += 1
    print(f"[ws cam{st.cam}] client connected {req.remote} (phones on this channel: {st.clients})", flush=True)
    try:
        async for msg in ws:
            if msg.type == WSMsgType.BINARY:
                await push_frame(st, msg.data)
            elif msg.type == WSMsgType.TEXT:
                print(f"[ws cam{st.cam}] {msg.data}", flush=True)
    finally:
        st.clients -= 1
        print(f"[ws cam{st.cam}] client disconnected", flush=True)
    return ws


# ---------------- local side (OpenCV / GUI) ----------------
def status_dict(st: State):
    return {"cam": st.cam, "phones": st.clients, "frames": st.count, "width": st.w, "height": st.h,
            "fps": round(st.fps, 1), "age_s": round(time.time() - st.last_ts, 2) if st.last_ts else None}


async def status(req):
    if "cam" in req.query:
        return web.json_response(status_dict(S[cam_of(req)]))
    d = {f"cam{c}": status_dict(S[c]) for c in CAMS}
    d.update({k: v for k, v in status_dict(S[1]).items() if k != "cam"})   # old flat keys = cam 1
    return web.json_response(d)


def latest_for(st):
    async def latest(req):
        if st.jpeg is None:
            return web.Response(status=503, text=f"no frame yet on cam {st.cam}\n")
        return web.Response(body=st.jpeg, content_type="image/jpeg", headers={"Cache-Control": "no-cache"})
    return latest


def stream_for(st):
    async def stream(req):
        resp = web.StreamResponse(headers={
            "Content-Type": "multipart/x-mixed-replace; boundary=frame",
            "Cache-Control": "no-cache", "Pragma": "no-cache"})
        await resp.prepare(req)
        last = -1
        try:
            while True:
                async with st.cond:
                    await st.cond.wait_for(lambda: st.count != last and st.jpeg is not None)
                b, last = st.jpeg, st.count
                await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(b)).encode() + b"\r\n\r\n" + b + b"\r\n")
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp
    return stream


def status_for(st):
    async def h(req):
        return web.json_response(status_dict(st))
    return h


async def main():
    for st in S.values():
        st.cond = asyncio.Condition()
    phone = web.Application(middlewares=[require_token])
    phone.add_routes([web.get("/", index), web.get("/ws", ws_handler), web.get("/status", status)])
    local = web.Application()
    routes = [web.get("/stream.mjpg", stream_for(S[1])), web.get("/latest.jpg", latest_for(S[1])),
              web.get("/snapshot.jpg", latest_for(S[1])), web.get("/", status), web.get("/status", status)]
    for c, st in S.items():
        routes += [web.get(f"/stream{c}.mjpg", stream_for(st)), web.get(f"/latest{c}.jpg", latest_for(st)),
                   web.get(f"/status{c}", status_for(st))]
    local.add_routes(routes)
    ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
    ctx.load_cert_chain(HERE / "cert.pem", HERE / "key.pem")
    r1 = web.AppRunner(phone); await r1.setup()
    await web.TCPSite(r1, "0.0.0.0", HTTPS_PORT, ssl_context=ctx).start()
    r2 = web.AppRunner(local); await r2.setup()
    await web.TCPSite(r2, "127.0.0.1", MJPEG_PORT).start()
    write_links(TOKEN)
    print(f"phone pages: see {LINKS_FILE} (token required)  |  local MJPEG: "
          f"http://127.0.0.1:{MJPEG_PORT}/stream1.mjpg, /stream2.mjpg", flush=True)
    while True:
        await asyncio.sleep(10)
        for st in S.values():
            if st.count:
                print(f"[stat] {json.dumps(status_dict(st))}", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
