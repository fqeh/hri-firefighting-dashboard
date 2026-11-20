import { defineConfig } from 'vite';

export default defineConfig({
  server: { host: '127.0.0.1', port: 5173, strictPort: true,
    proxy: { '/api/session': { target: 'http://127.0.0.1:8081', changeOrigin: false }, '/api': { target: 'http://127.0.0.1:8080', changeOrigin: false } } },
});
