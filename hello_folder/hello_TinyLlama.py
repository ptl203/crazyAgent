# hello_TinyLlama.py
# Simple web chat that talks to Ollama (TinyLlama) on your Raspberry Pi

from flask import Flask, request, jsonify, render_template_string
import requests

# === EDIT ME if your Pi IP changes ===
OLLAMA_HOST = "192.168.44.2"
OLLAMA_PORT = 11434
OLLAMA_CHAT_URL = f"http://{OLLAMA_HOST}:{OLLAMA_PORT}/api/chat"

app = Flask(__name__)

HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>TinyLlama Chat</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 2rem; }
    #log { white-space: pre-wrap; border: 1px solid #ddd; padding: 1rem; height: 50vh; overflow:auto; }
    input, button, select { font-size: 1rem; padding: .5rem; }
    .row { margin: .5rem 0; }
  </style>
</head>
<body>
  <h1>TinyLlama Chat (Ollama on {{ host }}:{{ port }})</h1>
  <div class="row">
    <label>Temperature: <input type="number" id="temp" value="0.7" step="0.1" min="0" max="2"></label>
    <label>Max tokens: <input type="number" id="tokens" value="128" min="1" max="2048"></label>
  </div>
  <div id="log"></div>
  <div class="row">
    <input id="msg" style="width:70%" placeholder="Type your message" />
    <button onclick="send()">Send</button>
    <button onclick="reset()">Reset Chat</button>
  </div>

<script>
let history = [
  {"role":"system","content":"You are a concise, helpful assistant."}
];

function append(text, who) {
  const log = document.getElementById('log');
  const p = document.createElement('div');
  p.textContent = (who ? who + ": " : "") + text;
  log.appendChild(p);
  log.scrollTop = log.scrollHeight;
}

function reset() {
  history = [{"role":"system","content":"You are a concise, helpful assistant."}];
  document.getElementById('log').textContent = "";
  append("Chat reset.", "system");
}

async function send() {
  const msgEl = document.getElementById('msg');
  const t = parseFloat(document.getElementById('temp').value);
  const n = parseInt(document.getElementById('tokens').value, 10);
  const text = msgEl.value.trim();
  if (!text) return;

  append(text, "you");
  msgEl.value = "";

  const payload = { message: text, temperature: t, num_predict: n };

  const res = await fetch("/chat", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(payload)
  });
  const data = await res.json();
  if (data.error) {
    append("Error: " + data.error, "system");
  } else {
    append(data.reply, "tinyllama");
  }
}
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML, host=OLLAMA_HOST, port=OLLAMA_PORT)

@app.route("/chat", methods=["POST"])
def chat():
    body = request.get_json(force=True) or {}
    user_msg = body.get("message", "").strip()
    temperature = float(body.get("temperature", 0.7))
    num_predict = int(body.get("num_predict", 128))

    if not user_msg:
        return jsonify({"error": "message is required"}), 400

    # Build a single-turn chat. If you want full history, keep an array and append turns.
    payload = {
        "model": "tinyllama",
        "messages": [
            {"role": "system", "content": "You are a concise, helpful assistant."},
            {"role": "user", "content": user_msg}
        ],
        "stream": False,
        "options": {
            "temperature": temperature,
            "num_predict": num_predict
        }
    }

    try:
        r = requests.post(OLLAMA_CHAT_URL, json=payload, timeout=180)
        r.raise_for_status()
        data = r.json()
        reply = (data.get("message") or {}).get("content", "").strip()
        if not reply:
            return jsonify({"error": "Empty reply from Ollama"}), 502
        return jsonify({"reply": reply})
    except requests.exceptions.RequestException as e:
        return jsonify({"error": f"Request to Ollama failed: {e}"}), 502

@app.route("/health")
def health():
    # Quick connectivity probe
    try:
        t = requests.get(f"http://{OLLAMA_HOST}:{OLLAMA_PORT}/api/tags", timeout=5)
        if t.ok:
            return jsonify({"ok": True, "tags": t.json()})
        return jsonify({"ok": False, "status": t.status_code, "body": t.text}), 500
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

if __name__ == "__main__":
    # Run your local web UI on port 8080
    app.run(host="0.0.0.0", port=8080, debug=False)
