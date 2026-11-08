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
  .log { margin-top:12px; background:var(--card); border-radius:8px; padding:12px; }
  .log h2 { font-size:15px; margin:0 0 4px; }
  .sum { color:var(--dim); font-size:13px; margin-bottom:8px; }
  .tablewrap { overflow-x:auto; }
  table { width:100%; border-collapse:collapse; font-size:14px; font-variant-numeric:tabular-nums; }
  th { text-align:left; color:var(--dim); font-weight:500; font-size:12px; padding:6px 8px; border-bottom:1px solid #2c333c; }
  td { padding:6px 8px; border-bottom:1px solid #222830; white-space:nowrap; }
  tr.active td { background:#22201c; }
  .st-active { color:var(--hot); font-weight:600; }
  .st-out { color:var(--ok); }
  .st-false, .st-other { color:var(--dim); }
  .empty { color:var(--dim); padding:8px; }
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
  <section class="log">
    <h2>Fire log</h2>
    <div class="sum" id="summary">No fires yet.</div>
    <div class="tablewrap"><table>
      <thead><tr><th>#</th><th>Sector</th><th>Detected</th><th>Status</th>
                 <th>Spraying after</th><th>Put out in</th></tr></thead>
      <tbody id="logrows"><tr><td colspan="6" class="empty">Waiting for the first fire...</td></tr></tbody>
    </table></div>
  </section>
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

// Our own queue instead of the browser's: events can come faster than speech, and the
// browser would read every old sentence in full, so the voice falls further and further
// behind. Here an announcement that has waited too long is skipped (the newest one is
// always kept), so the voice stays in step with what's happening right now.
const MAX_WAIT_MS = 2500;
const KEEP = ['detect', 'out', 'false', 'warn', 'clear'];   // never skipped (location / outcome)
const queue = [];
let speaking = false;

function speakNext() {
  if (!('speechSynthesis' in window)) { queue.length = 0; return; }
  const now = Date.now();
  // Skip stale filler (e.g. "confirmed") but never the newest item or anything in KEEP.
  for (let i = queue.length - 2; i >= 0; i--)
    if (!KEEP.includes(queue[i].kind) && now - queue[i].t > MAX_WAIT_MS) queue.splice(i, 1);
  const item = queue.shift();
  if (!item) { speaking = false; return; }
  speaking = true;
  if (item.chime) chime();
  const u = new SpeechSynthesisUtterance(item.text);
  if (voice) u.voice = voice;
  u.rate = 1.15;
  // Move on when the sentence ends, or after a time limit in case the browser never
  // reports the end (a known Chrome quirk) - otherwise the queue would get stuck.
  let done = false;
  const next = () => { if (!done) { done = true; clearTimeout(guard); speakNext(); } };
  const guard = setTimeout(next, 1200 + item.text.split(' ').length * 450);
  u.onend = u.onerror = next;
  window.__lastUtterance = u;          // keep a reference so Chrome doesn't drop the 'end' event
  speechSynthesis.speak(u);
}

function speak(text, withChime, kind) {
  queue.push({ text, chime: !!withChime, kind: kind || 'info', t: Date.now() });
  if (!speaking) speakNext();
}

btn.addEventListener('click', () => {
  voiceOn = !voiceOn;
  btn.textContent = voiceOn ? 'Voice on' : 'Enable voice';
  btn.classList.toggle('on', voiceOn);
  btn.setAttribute('aria-pressed', String(voiceOn));
  if (voiceOn) {
    try { audio = audio || new (window.AudioContext || window.webkitAudioContext)(); audio.resume(); } catch (e) {}
    speak(currentSituation() || 'Voice announcements on.', false, 'detect');
  } else {
    queue.length = 0; speaking = false;
    if ('speechSynthesis' in window) speechSynthesis.cancel();
  }
});

let latest = null;   // last /status reply
function currentSituation() {
  if (!latest || !latest.sector) return null;
  const sec = latest.sector.split('').join(' ');
  const what = { MOVE: 'Heading to fire in sector', CONFIRM: 'Checking sector',
                 SUPPRESS: 'Suppressing fire in sector', VERIFY: 'Verifying sector' }[latest.state];
  return what ? `Voice on. ${what} ${sec}.` : null;
}

// ---- fire log table ----
const fmt = v => (v === null || v === undefined) ? '-' : `${v.toFixed(1)} s`;
function statusClass(r) {
  if (r.active) return 'st-active';
  if (r.status === 'Out') return 'st-out';
  if (r.status === 'False alarm') return 'st-false';
  return 'st-other';
}
function esc(t) { const d = document.createElement('div'); d.textContent = t; return d.innerHTML; }
function renderLog(rows, sum) {
  const body = document.getElementById('logrows');
  if (!rows || !rows.length) return;
  body.innerHTML = rows.map(r => `<tr class="${r.active ? 'active' : ''}">
      <td>${r.n}</td><td>${esc(r.sector)}</td><td>${esc(r.detected)}</td>
      <td class="${statusClass(r)}">${esc(r.status)}${r.active ? ` (${Math.floor(r.elapsed)} s)` : ''}</td>
      <td>${fmt(r.response)}</td><td>${fmt(r.duration)}</td></tr>`).join('');
  if (sum && sum.out) {
    document.getElementById('summary').textContent =
      `Fires put out: ${sum.out}  |  Average time until spraying: ${fmt(sum.avg_response)}  |  ` +
      `Average time to put out: ${fmt(sum.avg_duration)}`;
  } else {
    document.getElementById('summary').textContent = 'No fires put out yet.';
  }
}

function handleEvents(events) {
  if (!events || !events.length) return;
  const newest = events[events.length - 1].id;
  if (lastId === null) { lastId = newest; return; }      // don't read out old history on page load
  for (const e of events) {
    if (e.id <= lastId) continue;
    lastId = e.id;
    document.getElementById('said').textContent = e.text;
    if (voiceOn) speak(e.text, ['detect', 'false', 'warn'].includes(e.kind), e.kind);
  }
}

async function poll() {
  try {
    const s = await (await fetch('/status', {cache:'no-store'})).json();
    const st = document.getElementById('state');
    st.textContent = s.state; st.className = s.state;
    for (const k of ['fires','target','flame','pos','out','false','alert'])
      document.getElementById(k).textContent = s[k];
    latest = s;
    handleEvents(s.events);
    renderLog(s.log, s.summary);
  } catch (e) {
    const st = document.getElementById('state');
    st.textContent = 'disconnected'; st.className = 'off';
  }
}
setInterval(poll, 300); poll();
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
