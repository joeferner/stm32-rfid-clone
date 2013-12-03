'use strict';

var SerialPort = require("serialport").SerialPort;

exports.init = function(app) {
  var serialPort = new SerialPort("/dev/ttyACM0", {
    baudrate: 57600
  });

  serialPort.on('error', function(err) {
    console.log('err', err);
  });

  serialPort.on('data', function(d) {
    var time = d.readUInt32LE(0);
    app.emitData([
      time
    ]);
  });
};
