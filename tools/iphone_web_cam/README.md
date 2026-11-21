# iPhone browser camera bridge (2 channels)

Turns up to two iPhones into cameras for the Mac without any app: Safari on the phone captures the camera
(getUserMedia over HTTPS) and sends JPEG frames over a WebSocket to `server.py`, which re-serves them locally as
MJPEG for OpenCV, LeRobot and the teleop GUI.

## Use
1. `./start.sh` (idempotent; creates the self-signed cert for the current LAN IP and the access token).
2. Open the links from **`links.txt`** (this folder, chmod 600) in Safari on the phones – the links contain the
   secret token:
   - iPhone 1 = link with `cam=1` → broad overview camera (GUI panel "Kamera 1")
   - iPhone 2 = link with `cam=2` → gripper camera (GUI panel "Kamera 2")
   A link without `cam=` means cam 1.
3. Accept the certificate warning once ("Show details → visit this website"), tap **Start**, allow camera access.
   Keep Safari in front and the screen on (the stream stops when the phone locks).
4. **Camera picker:** after the first Start (camera permission), the *Camera* dropdown lists every camera of
   the phone with its label (e.g. back, ultra-wide 0.5x, telephoto, front – depends on the model). *Resolution*
   (640×480 / 1280×720 default / 1920×1080) and *FPS* (15 / 24 / 30 default / 60) can be changed too. Changes
   apply immediately while streaming (camera is swapped, the connection stays). The choice is remembered per
   channel (cam 1 / cam 2) in the phone's browser storage. If a remembered camera is gone, the page falls back to
   the back camera. (Safari currently exposes only the 1× main camera.)
5. `./stop.sh` stops the server. After a server restart, reload the page on the phone and tap Start again.

## Local endpoints (127.0.0.1:8090 only, not reachable from the network)
| Path | Content |
|---|---|
| `/stream1.mjpg`, `/stream2.mjpg` | MJPEG per channel (`/stream.mjpg` = alias for cam 1) |
| `/latest1.jpg`, `/latest2.jpg` | newest JPEG per channel (`/latest.jpg`, `/snapshot.jpg` = cam 1) |
| `/status` | JSON for both channels (old flat keys = cam 1); `/status1`, `/status2` per channel |

Files: newest frames are also written ~5×/s to `/tmp/iphone_web_cam/latest.jpg` (cam 1) and `latest2.jpg` (cam 2).

OpenCV: `cv2.VideoCapture("http://127.0.0.1:8090/stream1.mjpg")`.

## Security
- The phone side (HTTPS :8444) requires `?t=<token>` on the page, `/status` and the WebSocket; otherwise **403**.
  The token is random, generated once in `token.txt` (chmod 600). To rotate it: `./stop.sh; rm token.txt; ./start.sh`
  and re-open the new links from `links.txt` on the phones.
- The MJPEG side listens on 127.0.0.1 only.
- Self-signed TLS → the browser warning is expected; the traffic is still encrypted.
- Checked 2026-11-15: nothing on the Mac listens on 80/443, and ports 8443/8444/8090 are not reachable from the
  internet (router at <lan-ip>, no forwarding to these ports seen).
- Port 8443 belongs to a different app ("Dave voice agent") – do not use it for the camera.

## Notes
- The phone sends 1280×720 at up to 30 fps (JPEG quality 0.8); rotate the phone to landscape for a wide image.
- If two phones open the same channel, the frames interleave – use one phone per channel.
- Backups of the single-channel version: `server_v1_single.py.bak`, `index_v1_single.html.bak`, `start_v1.sh.bak`.
