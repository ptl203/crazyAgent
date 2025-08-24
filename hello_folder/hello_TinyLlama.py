# hello_TinyLlama.py
# Simple Gradio chat that talks to Ollama (TinyLlama) on your Raspberry Pi

import gradio as gr
import requests

# === EDIT ME if your Pi IP changes ===
OLLAMA_HOST = "192.168.44.2"
OLLAMA_PORT = 11434
OLLAMA_CHAT_URL = f"http://{OLLAMA_HOST}:{OLLAMA_PORT}/api/chat"

# Global chat history
chat_history = []

def chat_with_tinyllama(message, history, temperature, max_tokens):
    """Send message to TinyLlama and return response"""
    if not message.strip():
        return history, ""
    
    # Add user message to history
    history = history + [[message, None]]
    
    payload = {
        "model": "tinyllama",
        "messages": [
            {"role": "system", "content": "You are a concise, helpful assistant."},
            {"role": "user", "content": message}
        ],
        "stream": False,
        "options": {
            "temperature": temperature,
            "num_predict": max_tokens
        }
    }
    
    try:
        r = requests.post(OLLAMA_CHAT_URL, json=payload, timeout=180)
        r.raise_for_status()
        data = r.json()
        reply = (data.get("message") or {}).get("content", "").strip()
        
        if not reply:
            reply = "Empty reply from Ollama"
        
        # Add bot response to history
        history[-1][1] = reply
        return history, ""
        
    except requests.exceptions.RequestException as e:
        error_msg = f"Request to Ollama failed: {e}"
        history[-1][1] = error_msg
        return history, ""

def reset_chat():
    """Reset chat history"""
    return [], ""

def check_health():
    """Check Ollama connectivity"""
    try:
        t = requests.get(f"http://{OLLAMA_HOST}:{OLLAMA_PORT}/api/tags", timeout=5)
        if t.ok:
            return f"✅ Connected to Ollama at {OLLAMA_HOST}:{OLLAMA_PORT}"
        return f"❌ Connection failed: Status {t.status_code}"
    except Exception as e:
        return f"❌ Connection failed: {e}"

# Create Gradio interface
with gr.Blocks(title="TinyLlama Chat") as demo:
    gr.Markdown(f"# TinyLlama Chat (Ollama on {OLLAMA_HOST}:{OLLAMA_PORT})")
    
    with gr.Row():
        with gr.Column(scale=1):
            temperature = gr.Slider(
                minimum=0.0, 
                maximum=2.0, 
                value=0.7, 
                step=0.1, 
                label="Temperature"
            )
            max_tokens = gr.Slider(
                minimum=1, 
                maximum=2048, 
                value=128, 
                step=1, 
                label="Max Tokens"
            )
            health_btn = gr.Button("Check Connection")
            health_status = gr.Textbox(
                value="Click 'Check Connection' to test",
                label="Connection Status",
                interactive=False
            )
        
        with gr.Column(scale=3):
            chatbot = gr.Chatbot(
                label="Chat with TinyLlama",
                height=400
            )
            msg = gr.Textbox(
                placeholder="Type your message here...",
                label="Message",
                lines=1
            )
            with gr.Row():
                submit = gr.Button("Send", variant="primary")
                clear = gr.Button("Reset Chat")
    
    # Event handlers
    def respond(message, history, temp, tokens):
        return chat_with_tinyllama(message, history, temp, tokens)
    
    submit.click(
        respond,
        inputs=[msg, chatbot, temperature, max_tokens],
        outputs=[chatbot, msg]
    )
    
    msg.submit(
        respond,
        inputs=[msg, chatbot, temperature, max_tokens],
        outputs=[chatbot, msg]
    )
    
    clear.click(
        reset_chat,
        outputs=[chatbot, msg]
    )
    
    health_btn.click(
        check_health,
        outputs=[health_status]
    )

if __name__ == "__main__":
    # Run Gradio interface
    demo.launch(
        server_name="0.0.0.0",
        server_port=8080,
        share=False
    )
