const express = require('express');
const WebSocket = require('ws');
const net = require('net');

const WEBSOCKET_PORT = 8675;
const UNIX_SOCKET_PATH = '/tmp/balance_bot.sock';

const app = express();
const wss = new WebSocket.Server({ port: WEBSOCKET_PORT });

let balanceBotSocket = null;

// MODULE SCOPE, deliberately. Everything from rxBuf down lives inside
// connectToBalanceBot(), which is fine for state that belongs to one IPC
// connection -- but the websocket 'connection' handler is out here, and it
// needs to read this. Declared inside, it was invisible from there and every
// browser connect threw ReferenceError, killing the socket: the dashboard
// showed "not connected" while every service was running normally. `node
// --check` cannot see that; it is a runtime scope error, not a syntax one.
//
// Keeping it out here also means the cached config survives an IPC reconnect,
// which is the behaviour we actually want.
let lastConfig = null;

function connectToBalanceBot() {
    console.log('[IPC] Connecting to balance_bot...');
    
    balanceBotSocket = net.createConnection(UNIX_SOCKET_PATH);
    
    balanceBotSocket.on('connect', function() {
        console.log('[IPC] Connected to balance_bot');
    });

// A unix socket is a byte stream, not a message queue. 'data' fires with
// whatever chunk sizes the kernel picks, so one event can carry half a packet,
// or two and a half. The previous version split each chunk on newlines and
// forwarded the pieces, which meant any packet straddling a chunk boundary was
// forwarded as two fragments -- both invalid JSON, both silently dropped by the
// browser's try/catch. It only appeared to work while packets were small enough
// to usually land in a single read; as telemetry grew, the drop rate climbed and
// the dashboard went erratic while the robot was perfectly fine.
//
// Accumulate instead, and emit only complete newline-terminated lines.
let rxBuf = '';
const RX_BUF_LIMIT = 1 << 20;   // 1 MB: a peer that never sends '\n' is broken,
                                // and unbounded growth would be a memory leak

// ── Backpressure ────────────────────────────────────────────────────────────
//
// ws.send() queues into an unbounded userspace buffer when the socket cannot
// take more. The previous broadcast checked only readyState === OPEN, which is
// not a liveness test: a phone that sleeps, changes networks or walks out of
// range leaves a socket in OPEN for minutes, because TCP does not find out
// until it times out. Every telemetry line was queued into that dead client
// forever, and with 20 Hz of telemetry plus 20 Hz of RC that is ~30 KB/s of
// permanent growth per stalled client. `botss` showed four connects against
// two disconnects, and node carrying 2.3 MB of swap.
//
// Telemetry is a stream of latest-state, not a log: a client that cannot keep
// up should MISS frames, never delay everyone else's. So drop for the slow
// client rather than queue. This is the same decision already made on the bot
// side of the socket, where ipc_server.c checks SIOCOUTQ before sending.
const SEND_HIGH_WATER = 256 * 1024;   // ~8 s of stream; past this the client is not draining
const STALL_EVICT_MS = 15000;         // over the mark this long => gone, not slow
let dropped = 0;
let lastDropLog = 0;

// The bot sends "config" ONCE, when a client connects to the unix socket —
// and that client is this bridge, not the browser. Node connects at startup,
// receives the config, forwards it to whoever happens to be attached at that
// instant, and never sees another unless a set_* command changes something.
//
// So every browser that connects later gets no config at all, and its Position
// Hold sliders sit at built-in defaults while claiming to show the robot. The
// dashboard's CSV preamble caught it honestly:
//   "config_source=UI DEFAULTS — bot never reported pos_config"
//
// Gains kept working throughout because they still ride in the telemetry
// packet, which is exactly why this looked like a partial failure rather than
// a missing message.
//
// The bridge is the fan-out point and already sees the message, so it is the
// right place to remember it. This also covers reconnects, which matter more
// than it first appears: a client that drops and comes back would otherwise
// lose its config every time.
//
// `lastConfig` itself is declared at module scope -- see the note there.

function broadcast(line) {
    if (line.startsWith('{"type":"config"')) lastConfig = line;
    const now = Date.now();
    wss.clients.forEach(function (client) {
        if (client.readyState !== WebSocket.OPEN) return;

        if (client.bufferedAmount > SEND_HIGH_WATER) {
            if (!client._stalledSince) client._stalledSince = now;
            // A client that has been backed up this long is not slow, it is
            // gone. Terminate it so its buffer is actually released instead of
            // sitting in the heap until TCP eventually gives up.
            if (now - client._stalledSince > STALL_EVICT_MS) {
                console.error('[WebSocket] evicting stalled client (' +
                              client.bufferedAmount + ' bytes queued)');
                client.terminate();
                return;
            }
            dropped++;
            if (now - lastDropLog > 5000) {
                console.error('[WebSocket] dropping frames for a slow client: ' +
                              dropped + ' so far, ' + client.bufferedAmount +
                              ' bytes queued');
                lastDropLog = now;
            }
            return;
        }
        client._stalledSince = null;
        client._sent = (client._sent || 0) + 1;
        client.send(line);
    });
}

balanceBotSocket.on('data', function (data) {
    rxBuf += data.toString();

    let nl;
    while ((nl = rxBuf.indexOf('\n')) !== -1) {
        const line = rxBuf.slice(0, nl).trim();
        rxBuf = rxBuf.slice(nl + 1);
        if (!line) continue;
        broadcast(line);
    }

    if (rxBuf.length > RX_BUF_LIMIT) {
        console.error('[IPC] no delimiter in ' + rxBuf.length + ' bytes, discarding');
        rxBuf = '';
    }
});

balanceBotSocket.on('close', function () { rxBuf = ''; });
    
    
    balanceBotSocket.on('error', function(err) {
        console.error('[IPC] Error:', err.message);
    });
    
    balanceBotSocket.on('close', function() {
        console.log('[IPC] Disconnected');
        balanceBotSocket = null;
        setTimeout(connectToBalanceBot, 5000);
    });
}

// ── Liveness ────────────────────────────────────────────────────────────────
//
// readyState only tells you the socket has not been closed, which a sleeping
// phone's socket never is. Ping/pong is the only way to find out whether the
// far end still exists. Without this the client list grows monotonically
// across a day of picking the phone up and putting it down.
const HEARTBEAT_MS = 15000;

// ── Per-client status ───────────────────────────────────────────────────────
//
// One number per client, because the aggregate cannot answer the question that
// matters. "The dashboard is getting 2.4 Hz" is compatible with two completely
// different faults:
//
//   * this bridge is not forwarding      -> EVERY client is slow, including a
//                                           loopback one with no network in the
//                                           path
//   * one consumer cannot keep up        -> that client is slow and every other
//                                           client is fine
//
// From node those look identical unless you break the number out per client,
// so this prints delivered rate and queued bytes for each one. Run
// `./tools/ws_rate.py 20 ws://localhost:8675` ON THE BOT with the browser also
// open, and this line says outright which of the two it is.
//
// Note also that a full send buffer is NOT proof the network is broken: TCP
// flow control produces exactly the same backlog when the far end simply stops
// reading. What distinguishes them is whether a *different* client on the same
// bridge stays healthy.
const STATUS_MS = 5000;
let lastStatusHealthy = false;

setInterval(function () {
    if (wss.clients.size === 0) { lastStatusHealthy = false; return; }
    const parts = [];
    let healthy = true;
    wss.clients.forEach(function (c) {
        const hz = (c._sent || 0) / (STATUS_MS / 1000);
        c._sent = 0;
        const buf = c.bufferedAmount;
        const who = (c._socket && c._socket.remoteAddress) || '?';
        parts.push(who + ' ' + hz.toFixed(1) + 'Hz buf=' + buf);
        if (buf > 0 || hz < 15) healthy = false;
    });
    // Chatter only while something is wrong; one line on the way back to
    // healthy so the journal shows recovery rather than just going quiet.
    if (!healthy) {
        console.warn('[WebSocket] status: ' + parts.join(' | '));
        lastStatusHealthy = false;
    } else if (!lastStatusHealthy) {
        console.log('[WebSocket] status: ' + parts.join(' | ') + '  (healthy)');
        lastStatusHealthy = true;
    }
}, STATUS_MS);

const heartbeat = setInterval(function () {
    wss.clients.forEach(function (client) {
        if (client.isAlive === false) {
            console.log('[WebSocket] client failed heartbeat, terminating');
            return client.terminate();
        }
        client.isAlive = false;
        try { client.ping(); } catch (e) { /* terminating anyway next round */ }
    });
}, HEARTBEAT_MS);

wss.on('close', function () { clearInterval(heartbeat); });

wss.on('connection', function(ws) {
    console.log('[WebSocket] client connected (' + (wss.clients.size) + ' total)');
    ws.isAlive = true;
    ws._stalledSince = null;
    ws.on('pong', function () { ws.isAlive = true; });

    // Replay the last config to this client before any telemetry reaches it,
    // so its controls are seeded from the robot rather than from defaults.
    if (lastConfig) {
        try {
            ws.send(lastConfig);
        } catch (e) {
            console.error('[WebSocket] could not replay config: ' + e.message);
        }
    } else {
        // Nothing cached means this bridge has not seen a config since it
        // started. Say so — silence here previously looked identical to a
        // client that simply had not asked for one.
        console.warn('[WebSocket] no cached config to replay — ' +
                     'client will show defaults until the bot sends one');
    }

    ws.on('error', function (err) {
        console.error('[WebSocket] client error: ' + err.message);
    });

    ws.on('message', function(message) {
        if (balanceBotSocket && balanceBotSocket.writable) {
            balanceBotSocket.write(message + '\n');
        }
    });
    
    ws.on('close', function() {
        // ws removes the client from wss.clients BEFORE firing 'close', so the
        // size is already correct here. Subtracting one printed "-1 left".
        console.log('[WebSocket] client disconnected (' + wss.clients.size + ' left)');
    });
});

app.get('/status', function(req, res) {
    res.json({
        server: 'Cat Follower Server',
        websocket: { port: WEBSOCKET_PORT, clients: wss.clients.size },
        balanceBot: { connected: balanceBotSocket && balanceBotSocket.writable }
    });
});

app.listen(3141, function() {
    console.log('[HTTP] Status server on port 3141');
    console.log('[WebSocket] Server on port 8675');
    connectToBalanceBot();
});
