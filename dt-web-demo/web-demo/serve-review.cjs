/* Loopback-only preview for the two independent working trees. No build or dependencies. */
'use strict';
const fs = require('node:fs');
const http = require('node:http');
const path = require('node:path');
const {spawn} = require('node:child_process');
const optimized = __dirname;
const current = path.resolve(__dirname, '../../低空专项/web-demo');
const argument = process.argv.find(value => /^--port=/.test(value));
const port = argument ? Number(argument.split('=')[1]) : 8768;
if (!Number.isInteger(port) || port < 1024 || port > 65535) throw new Error('Port must be between 1024 and 65535.');
const types = {'.html':'text/html; charset=utf-8','.css':'text/css; charset=utf-8','.js':'application/javascript; charset=utf-8','.svg':'image/svg+xml','.png':'image/png','.json':'application/json; charset=utf-8'};
const server = http.createServer((req, res) => {
  let pathname;
  try { pathname = decodeURIComponent(new URL(req.url, 'http://localhost').pathname); }
  catch { res.writeHead(400).end('Invalid path'); return; }
  if (pathname === '/current') { res.writeHead(302, {Location:'/current/'}).end(); return; }
  const old = pathname.startsWith('/current/');
  const root = old ? current : optimized;
  const relative = old ? pathname.slice('/current'.length) : pathname;
  const file = path.resolve(root, '.' + (relative.endsWith('/') ? relative + 'index.html' : relative));
  if (!file.startsWith(root + path.sep)) { res.writeHead(403).end('Forbidden'); return; }
  if (req.method !== 'GET' && req.method !== 'HEAD') { res.writeHead(405, {Allow:'GET, HEAD'}).end(); return; }
  fs.readFile(file, (error, data) => {
    if (error) { res.writeHead(404).end('Not found'); return; }
    res.writeHead(200, {'Content-Type': types[path.extname(file)] || 'application/octet-stream', 'Cache-Control':'no-store'});
    res.end(req.method === 'HEAD' ? undefined : data);
  });
});
server.on('error', error => {
  console.error(error.code === 'EADDRINUSE' ? 'Port '+port+' is in use. Choose another with --port=8769.' : error.message);
  process.exitCode = 1;
});
server.listen(port, '127.0.0.1', () => {
  const url = 'http://127.0.0.1:' + port + '/';
  console.log('Optimized: ' + url);
  console.log('Current:   ' + url + 'current/');
  console.log('Press Ctrl+C to stop both previews. Original source is read only.');
  if (process.argv.includes('--open') && process.platform === 'win32') {
    for (const link of [url + 'current/', url]) spawn('cmd.exe', ['/c', 'start', '', link], {windowsHide:true, stdio:'ignore'}).unref();
  }
});
for (const signal of ['SIGINT', 'SIGTERM']) process.on(signal, () => server.close());
