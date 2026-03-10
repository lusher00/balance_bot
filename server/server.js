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

balanceBotSocket.on('data', function(data) {
    const messages = data.toString().split('\n').filter(function(m) { return m.trim(); });
    messages.forEach(function(message) {
        wss.clients.forEach(function(client) {
            if (client.readyState === WebSocket.OPEN) {
                client.send(message);
            }
        });
    });
});
    
    
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
