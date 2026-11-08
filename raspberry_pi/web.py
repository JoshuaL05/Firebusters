"""
web.py - live view in a web browser.

    python3 main.py --web          (then open http://heatseek.local:8080 on any device
                                    on the same Wi-Fi: laptop, phone, projector...)

The Pi serves a small page with the live heat map (fire boxes, priority labels,
nozzle marker) and the current state. No extra packages: just Python's built-in
web server and OpenCV's JPEG encoder.
"""

import json
import logging
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

log = logging.getLogger("web")

PAGE = """<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>FireBusters Live</title>
<style>
  :root { --bg:#111418; --card:#1b2027; --text:#e8eaed; --dim:#9aa0a6; --hot:#ff6a3d; --ok:#3ddc84; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--text);
         font:16px/1.4 system-ui, -apple-system, Segoe UI, Roboto, sans-serif; }
  main { max-width:900px; margin:0 auto; padding:16px; }
  h1 { font-size:22px; margin:4px 0 12px; }
  .view { width:100%; border-radius:10px; background:#000; display:block; image-rendering:pixelated; }
  .bar { display:flex; flex-wrap:wrap; gap:8px; margin-top:12px; }
  .chip { background:var(--card); border-radius:8px; padding:8px 12px; }
  .chip b { display:block; font-size:12px; color:var(--dim); font-weight:500; }
  #state { font-weight:700; }
  .SUPPRESS, .ESTOP { color:var(--hot); }
  .SCAN { color:var(--ok); }
  .off { color:var(--dim); }
  .top { display:flex; align-items:center; justify-content:space-between; gap:12px; flex-wrap:wrap; margin:4px 0 12px; }
  .top h1 { margin:0; }
  button { font:inherit; color:var(--text); background:var(--card); border:1px solid #2c333c;
           border-radius:8px; padding:8px 14px; cursor:pointer; }
  button.on { border-color:var(--ok); color:var(--ok); }
  .say { margin-top:12px; background:var(--card); border-radius:8px; padding:10px 12px; min-height:44px; }
  .say b { font-size:12px; color:var(--dim); font-weight:500; margin-right:8px; }
</style></head>
<body><main>
  <div class="top">
    <h1>FireBusters: live thermal view</h1>
    <button id="voice" type="button" aria-pressed="false">Enable voice</button>
  </div>
  <img class="view" src="/stream" alt="live thermal camera">
  <div class="bar">
    <div class="chip"><b>State</b><span id="state">...</span></div>
    <div class="chip"><b>Fires seen</b><span id="fires">-</span></div>
    <div class="chip"><b>Target</b><span id="target">-</span></div>
    <div class="chip"><b>Flame sensor</b><span id="flame">-</span></div>
    <div class="chip"><b>Nozzle (mm)</b><span id="pos">-</span></div>
    <div class="chip"><b>Fires put out</b><span id="out">-</span></div>
    <div class="chip"><b>False alarms</b><span id="false">-</span></div>
    <div class="chip"><b>Fire dept. call</b><span id="alert">-</span></div>
  </div>
  <div class="say" aria-live="polite"><b>Announcement</b><span id="said">-</span></div>
</main>
<script>
// ---- voice narration (browser text-to-speech, free, nothing to install) ----
// Browsers only allow sound after a click on the page, hence the button.
let voiceOn = false, lastId = null, audio = null, voice = null;
const btn = document.getElementById('voice');

function pickVoice() {
  const vs = speechSynthesis.getVoices();
  voice = vs.find(v => /^en(-|_)(US|GB|CA)/i.test(v.lang) && /natural|google|samantha|aria|jenny/i.test(v.name))
       || vs.find(v => /^en/i.test(v.lang)) || null;
}
if ('speechSynthesis' in window) { pickVoice(); speechSynthesis.onvoiceschanged = pickVoice; }

function chime() {                       // short two-tone alert before important announcements
  if (!audio) return;
  const t = audio.currentTime;
  [[880, 0], [660, 0.16]].forEach(([f, d]) => {
    const o = audio.createOscillator(), g = audio.createGain();
    o.frequency.value = f; o.type = 'sine';
    g.gain.setValueAtTime(0.0001, t + d);
    g.gain.exponentialRampToValueAtTime(0.25, t + d + 0.02);
    g.gain.exponentialRampToValueAtTime(0.0001, t + d + 0.14);
    o.connect(g).connect(audio.destination); o.start(t + d); o.stop(t + d + 0.15);
  });
}

function speak(text) {
  if (!('speechSynthesis' in window)) return;
  const u = new SpeechSynthesisUtterance(text);
  if (voice) u.voice = voice;
  u.rate = 1.05;
  speechSynthesis.speak(u);
}

btn.addEventListener('click', () => {
  voiceOn = !voiceOn;
  btn.textContent = voiceOn ? 'Voice on' : 'Enable voice';
  btn.classList.toggle('on', voiceOn);
  btn.setAttribute('aria-pressed', String(voiceOn));
  if (voiceOn) {
    try { audio = audio || new (window.AudioContext || window.webkitAudioContext)(); audio.resume(); } catch (e) {}
    speak('Voice announcements on.');
  } else if ('speechSynthesis' in window) {
    speechSynthesis.cancel();
  }
});

function handleEvents(events) {
  if (!events || !events.length) return;
  const newest = events[events.length - 1].id;
  if (lastId === null) { lastId = newest; return; }      // don't read out old history on page load
  for (const e of events) {
    if (e.id <= lastId) continue;
    lastId = e.id;
    document.getElementById('said').textContent = e.text;
    if (voiceOn) {
      if (['detect', 'false', 'warn'].includes(e.kind)) chime();
      speak(e.text);
    }
  }
}

async function poll() {
  try {
    const s = await (await fetch('/status', {cache:'no-store'})).json();
    const st = document.getElementById('state');
    st.textContent = s.state; st.className = s.state;
    for (const k of ['fires','target','flame','pos','out','false','alert'])
      document.getElementById(k).textContent = s[k];
    handleEvents(s.events);
  } catch (e) {
    const st = document.getElementById('state');
    st.textContent = 'disconnected'; st.className = 'off';
  }
}
setInterval(poll, 500); poll();
</script>
</body></html>
"""


class WebView:
    def __init__(self, port=8080):
        self.port = port
        self._jpeg = None
        self._status = {}
        self._cond = threading.Condition()
        view = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):        # keep the console quiet
                pass

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._send(200, "text/html; charset=utf-8", PAGE.encode())
                elif self.path.startswith("/status"):
                    with view._cond:
                        body = json.dumps(view._status).encode()
                    self._send(200, "application/json", body)
                elif self.path.startswith("/stream"):
                    self._stream()
                else:
                    self._send(404, "text/plain", b"not found")

            def _send(self, code, ctype, body):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _stream(self):
                # MJPEG: the browser keeps the connection open and shows each new frame.
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last = None
                try:
                    while True:
                        with view._cond:
                            view._cond.wait_for(lambda: view._jpeg is not None and view._jpeg is not last,
                                                timeout=2.0)
                            jpeg = view._jpeg
                        if jpeg is None or jpeg is last:
                            continue
                        last = jpeg
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                                         + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass                          # viewer closed the page

        self._server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        host = socket.gethostname()
        log.info("Live view: on this computer open http://localhost:%d  |  from other devices on the "
                 "same Wi-Fi: http://%s.local:%d (or this computer's IP)", port, host.lower(), port)

    def update(self, frame_bgr, status):
        ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        with self._cond:
            self._jpeg = buf.tobytes()
            self._status = status
            self._cond.notify_all()

    def close(self):
        self._server.shutdown()
