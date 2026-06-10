// net.js — WebSocket channels to the backend (ICD §1): /ops and /eval,
// JSON {type, data}, reconnect with exponential backoff on drop.

export class Channel {
  constructor(url, { onMessage, onStatus } = {}) {
    this.url = url;
    this.onMessage = onMessage;
    this.onStatus = onStatus;
    this.backoff = 1000;
    this.closed = false;
    this.queue = [];          // control messages buffered while reconnecting
    this.ws = null;
    this._connect();
  }

  _connect() {
    if (this.closed) return;
    let ws;
    try {
      ws = new WebSocket(this.url);
    } catch (e) {
      this._scheduleReconnect();
      return;
    }
    this.ws = ws;
    ws.onopen = () => {
      this.backoff = 1000;
      this.onStatus?.(true);
      while (this.queue.length && ws.readyState === WebSocket.OPEN)
        ws.send(this.queue.shift());
    };
    ws.onmessage = (m) => {
      let msg;
      try { msg = JSON.parse(m.data); } catch (e) { return; }
      if (msg && typeof msg.type === 'string') this.onMessage?.(msg);
    };
    ws.onclose = () => {
      if (this.ws !== ws) return;
      this.onStatus?.(false);
      this._scheduleReconnect();
    };
    ws.onerror = () => { try { ws.close(); } catch (e) { /* noop */ } };
  }

  _scheduleReconnect() {
    if (this.closed) return;
    setTimeout(() => this._connect(), this.backoff);
    this.backoff = Math.min(this.backoff * 1.7, 15000);
  }

  send(msg) {
    const s = JSON.stringify(msg);
    if (this.ws && this.ws.readyState === WebSocket.OPEN) this.ws.send(s);
    else {
      this.queue.push(s);
      if (this.queue.length > 30) this.queue.shift();
    }
  }

  close() {
    this.closed = true;
    try { this.ws?.close(); } catch (e) { /* noop */ }
  }
}
