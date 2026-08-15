#!/usr/bin/env node
// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Ryan Lush <ryan.lush@gmail.com>
//
// ws_rate.js — measure the telemetry rate over the WebSocket, without a browser.
//
// sock_rate.py measures the C process. This measures what comes out the other
// side of server.js. Run it on the BOT (ws://localhost:8675) to test the bridge
// with the network removed, then from the MAC (ws://<bot-ip>:8675) to test the
// network. Whichever step loses the packets is the one at fault.
//
//   node tools/ws_rate.js                          # localhost, 5 s
//   node tools/ws_rate.js ws://192.168.1.142:8675 10
//
// Needs the ws module, which server/node_modules already has.

const path = require('path');
let WebSocket;
for (const p of ['ws', path.join(__dirname, '..', 'server', 'node_modules', 'ws')]) {
  try { WebSocket = require(p); break; } catch (e) { /* try next */ }
}
if (!WebSocket) {
  console.error("  cannot find the 'ws' module. Run from the repo root, or:");
  console.error('  cd server && npm install');
  process.exit(1);
}

const url = process.argv[2] || 'ws://localhost:8675';
const dur = (parseFloat(process.argv[3]) || 5) * 1000;

console.log(`  connecting to ${url} …`);
const ws = new WebSocket(url);

let n = 0, bytes = 0, bad = 0, last = 0;
const gaps = [];
let t0 = 0;

const timer = setTimeout(() => finish(), dur + 3000);   // hard stop

ws.on('open', () => {
  t0 = Date.now();
  last = t0;
  console.log('  connected, sampling…');
  setTimeout(finish, dur);
});

ws.on('message', (data) => {
  const now = Date.now();
  bytes += data.length;
  let ok = true;
  try { JSON.parse(data.toString()); } catch (e) { ok = false; bad++; }
  if (ok) {
    if (n) gaps.push(now - last);
    last = now;
    n++;
  }
});

ws.on('error', (e) => { console.error('  socket error:', e.message); process.exit(1); });
ws.on('close', () => finish());

let done = false;
function finish() {
  if (done) return;
  done = true;
  clearTimeout(timer);
  const el = (Date.now() - t0) / 1000 || 1;
  console.log(`\n  ${n} valid packets in ${el.toFixed(1)}s = ${(n / el).toFixed(1)} Hz`);
  console.log(`  ${(bytes / 1024).toFixed(1)} kB = ${(bytes / el / 1024).toFixed(1)} kB/s`);
  if (bad) console.log(`  ${bad} messages FAILED to parse  <- framing still broken`);
  if (gaps.length) {
    gaps.sort((a, b) => a - b);
    console.log(`  gap: min ${gaps[0]} ms, median ${gaps[gaps.length >> 1]} ms, max ${gaps[gaps.length - 1]} ms`);
  }
  console.log();
  if (n / el >= 8 && !bad) {
    console.log('  This hop is healthy.');
  } else if (bad) {
    console.log('  Packets are arriving but not parsing: framing is still wrong.');
  } else {
    console.log('  This hop is LOSING packets. The fault is at or before it.');
  }
  try { ws.close(); } catch (e) {}
  process.exit(0);
}
