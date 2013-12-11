'use strict';

var SerialPort = require("serialport").SerialPort;

exports.init = function(app) {
  var serialPort = new SerialPort("/dev/ttyACM0", {
    baudrate: 57600
  });

  serialPort.on('error', function(err) {
    console.log('serial port error', err);
    app.emitError(err);
  });

  serialPort.on('data', function(d) {
    //console.log(d.length, d);
    for (var i = 0; i < d.length - 12;) {
      var header = d.readUInt32LE(i);
      if (header != 0x01020304) {
        i++;
        continue;
      }
      var duration = d.readUInt32LE(i + 4);
      var bit = d.readUInt8(i + 8);
      i += 12;
      console.log("bit:", bit, "duration:", duration);
//      app.emitData([
//        duration,
//        bit
//      ]);
    }
  });
};
