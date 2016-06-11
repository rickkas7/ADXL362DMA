// Run this like:
// node server.js

var net = require('net');

// dataPort is the TCP port we listen on from the Photon. This value is encoded in the Photon code
var dataPort = 7123;

var rcvdTotal = 0;
var rcvdPeriod = 0;

// Start a TCP Server. This is what receives data from the Particle Photon
// https://gist.github.com/creationix/707146
net.createServer(function (socket) {
	console.log('data connection started from ' + socket.remoteAddress);
	
	// The server sends a 8-bit byte value for each sample. Javascript doesn't really like
	// binary values, so we use setEncoding to read each byte of a data as 2 hex digits instead.
	socket.setEncoding('hex');
	
	socket.on('data', function (data) {
		var count = data.length / 2;
		rcvdTotal += count;
		rcvdPeriod += count;
		// console.log("got " + count + " bytes rcvdPeriod=" + rcvdPeriod + " " + data.substr(0, 36));
	});
	socket.on('end', function () {
		console.log('data connection ended');
	});
}).listen(dataPort);


setInterval(function () {
	console.log("received " + rcvdPeriod + " bytes in the last 10 seconds " + rcvdTotal + " total");
	rcvdPeriod = 0;
}, 10000);

console.log('listening on port ' + dataPort + ' for data');
