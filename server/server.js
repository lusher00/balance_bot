const express = require('express');
const WebSocket = require('ws');
const net = require('net');

const WEBSOCKET_PORT = 8675;
const UNIX_SOCKET_PATH = '/tmp/balance_bot.sock';

const app = express();
const wss = new WebSocket.Server({ port: WEBSOCKET_PORT });

let balanceBotSocket = null;

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

balanceBotSocket.on('data', function (data) {
    rxBuf += data.toString();

    let nl;
    while ((nl = rxBuf.indexOf('\n')) !== -1) {
        const line = rxBuf.slice(0, nl).trim();
        rxBuf = rxBuf.slice(nl + 1);
        if (!line) continue;
        wss.clients.forEach(function (client) {
            if (client.readyState === WebSocket.OPEN) {
                client.send(line);
            }
        });
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

wss.on('connection', function(ws) {
    console.log('[WebSocket] iPhone connected');
    
    ws.on('message', function(message) {
        if (balanceBotSocket && balanceBotSocket.writable) {
            balanceBotSocket.write(message + '\n');
        }
    });
    
    ws.on('close', function() {
        console.log('[WebSocket] iPhone disconnected');
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
