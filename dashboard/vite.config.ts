import { defineConfig } from 'vite';

// /gazebo proxies the container's noVNC (port 6080), including its websocket,
// so the dashboard can embed the native Gazebo GUI through a single port.
const proxy = { '/api/session': { target: 'http://127.0.0.1:8081', changeOrigin: false }, '/api': { target: 'http://127.0.0.1:8080', changeOrigin: false },
  '/gazebo': { target: 'http://127.0.0.1:6080', changeOrigin: false, ws: true, rewrite: (path: string) => path.replace(/^\/gazebo/, '') } };

export default defineConfig({
  server: { host: '127.0.0.1', port: 5173, strictPort: true, proxy },
  // Built dashboard for remote viewers (scripts/run_public.sh): same proxies,
  // reachable only through a Cloudflare quick tunnel (*.trycloudflare.com).
  preview: { host: '127.0.0.1', port: 4173, strictPort: true, proxy, allowedHosts: ['.trycloudflare.com'] },
});
