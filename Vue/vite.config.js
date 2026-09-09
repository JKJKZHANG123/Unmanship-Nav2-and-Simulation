import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

// The monitor is served on the Jetson and accessed from a laptop browser on
// the same LAN.  Bind to all interfaces so it is reachable at
// http://<jetson-ip>:5173 (and proxy /ws to rosbridge on 9090 for dev).
export default defineConfig({
  plugins: [vue()],
  server: {
    host: '0.0.0.0',
    port: 5173,
    proxy: {
      '/rosbridge': {
        target: 'ws://127.0.0.1:9090',
        ws: true,
      },
    },
  },
})
