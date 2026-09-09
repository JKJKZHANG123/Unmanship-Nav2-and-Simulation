// rosbridge WebSocket 客户端。
// 直接实现 rosbridge v2 协议（subscribe / unadvertise），不引入 roslibjs，
// 这样对连接生命周期、压缩、超时完全可控。

export class RosbridgeClient {
  constructor(url) {
    this.url = url
    this.ws = null
    this.connected = false
    this.reconnectDelay = 2000
    this.handlers = new Map()   // topic -> Set<callback>
    this.pendingSubs = new Set()
    this.onStatus = null        // (status, detail) => void
    this._closedByUser = false
    this._serviceCalls = new Map()
    this._svcSeq = 0
    this.topicTypes = {}
  }

  connect() {
    this._closedByUser = false
    this._open()
  }

  _open() {
    if (this.ws) return
    let ws
    try {
      ws = new WebSocket(this.url)
    } catch (e) {
      this._scheduleReconnect()
      return
    }
    this.ws = ws

    ws.onopen = () => {
      this.connected = true
      if (this.onStatus) this.onStatus('connected', this.url)
      // 重连后重新订阅所有话题
      const topics = Array.from(new Set([
        ...this.pendingSubs,
        ...this.handlers.keys(),
      ]))
      for (const t of topics) this._sendSubscribe(t)
    }

    ws.onmessage = (evt) => this._onMessage(evt)

    ws.onclose = () => {
      this.connected = false
      this.ws = null
      if (this.onStatus) this.onStatus('disconnected', '')
      if (!this._closedByUser) this._scheduleReconnect()
    }

    ws.onerror = () => {
      // onclose 会随后触发，这里不额外处理
    }
  }

  _scheduleReconnect() {
    if (this._closedByUser) return
    setTimeout(() => this._open(), this.reconnectDelay)
  }

  _send(obj) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(obj))
    }
  }

  _sendSubscribe(topic) {
    this._send({
      op: 'subscribe',
      topic,
      type: this.topicTypes[topic],
      compression: 'none',
      throttle_rate: 0,
    })
  }

  setTopicTypes(map) {
    this.topicTypes = map
  }

  // 订阅。返回取消函数。
  subscribe(topic, callback) {
    if (!this.handlers.has(topic)) this.handlers.set(topic, new Set())
    this.handlers.get(topic).add(callback)
    this.pendingSubs.add(topic)
    if (this.connected) this._sendSubscribe(topic)
    return () => {
      const set = this.handlers.get(topic)
      if (set) {
        set.delete(callback)
        if (set.size === 0) this.handlers.delete(topic)
      }
    }
  }

  _onMessage(evt) {
    let msg
    try {
      msg = JSON.parse(evt.data)
    } catch (e) {
      return
    }
    if (msg.op === 'publish' && msg.topic) {
      const set = this.handlers.get(msg.topic)
      if (set) {
        for (const cb of set) cb(msg.msg, msg.topic)
      }
    }
    if (msg.op === 'service_response') {
      const pending = this._serviceCalls.get(msg.id)
      if (pending) {
        this._serviceCalls.delete(msg.id)
        pending.resolve(msg.values)
      }
    }
  }

  // 调用 ROS 服务。返回 Promise，resolve 服务返回的 values 对象。
  callService(service, args = {}) {
    return new Promise((resolve, reject) => {
      const id = 'srv_' + (++this._svcSeq)
      this._serviceCalls.set(id, { resolve, reject })
      this._send({
        op: 'call_service',
        id,
        service,
        args,
      })
      // 超时保护
      setTimeout(() => {
        if (this._serviceCalls.has(id)) {
          this._serviceCalls.delete(id)
          reject(new Error(`service ${service} timeout`))
        }
      }, 5000)
    })
  }

  close() {
    this._closedByUser = true
    if (this.ws) this.ws.close()
    this.ws = null
    this.connected = false
  }
}
