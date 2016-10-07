var fs = require('fs');
var csvparse = require('csv-parse');

module.exports = function (input, callback) {

    var source = input.source;
    var drain = input.drain;
    var drainTemp = input.drainTemp;
    var coords = input.coords;

    var tmx = coords[0], tmy = coords[1];
    var w = 1001, h = 1001;

    var buffer = Buffer.alloc(w*h*4);
    
    var readStream = fs.createReadStream(source);
    var parser = csvparse({delimiter: ';'});
    parser.on('readable', function() {
        var record;
        while ((record = parser.read()) !== null) {
            var xi = Math.ceil(record[0]);
            var yi = Math.ceil(record[1]);
            var x = Math.round(xi - tmx);
            var y = Math.round(h - 1 - (yi - tmy));
            if (x < 0 || x >= w || y < 0 || y >= h) {
                readStream.destroy();
                parser.resume();
                callback(null, {
                    err: source + " has invalid DMR grid: " + x + ", " + y
                });
                return;
            }
            var index = x + y*w;
            var height = Math.round(record[2]*100);
            buffer.writeInt32LE(height, index*4);
        }
    });
    parser.on('error', function(err) {
        callback(null, { err: "csv parser error: " + err });
    });
    parser.on('finish', function() {
        var dstream = fs.createWriteStream(drain);
        dstream.write(buffer, function() {
            dstream.close();
            fs.rename(drainTemp, drain, function renamedDrain(err) {
                callback(null, {});
            });
        });
    });
    readStream.pipe(parser);

};