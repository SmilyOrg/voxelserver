var Parser = require('node-dbf');
var fs = require('fs');
var http = require('http');
var path = require('path');
var mkdirp = require('mkdirp');
var async = require('async');
var exec = require('child_process').exec;
var Mustache = require('mustache');

var minx = 457262.46; var miny = 96189.57;
var maxx = 467601.05; var maxy = 106686.92;

var found = [];

var lidarRemote = "http://gis.arso.gov.si/";
var lidarLocal = "W:/gis/arso/";

var liberator = lidarLocal + "lasliberate/bin/lasliberate.exe";

// var configGKOT = { type: "gkot", prefix: "TM_" };
// var configOTR  = { type: "otr", prefix: "TMR_" };
var configGKOT = {
    remote: lidarRemote + "lidar/gkot/{{record.BLOK}}/D96TM/TM_{{record.NAME}}.zlas",
    local:  lidarLocal  + "lidar/gkot/{{record.BLOK}}/D96TM/TM_{{record.NAME}}.zlas",
    laz:    lidarLocal  + "laz/gkot/{{record.BLOK}}/D96TM/TM_{{record.NAME}}.laz"
}
var configDOF84 = {
    remote: lidarRemote +
        "arcgis/rest/services/opensource_dof84/MapServer/" +
        "export?bbox={{bounds.min.x}},{{bounds.min.y}},{{bounds.max.x}},{{bounds.max.y}}" + 
        "&bboxSR=3794&layers=&layerDefs=" +
        "&size={{image.width}},{{image.height}}" + 
        "&imageSR=3794&format=png&transparent=false&dpi=&time=&layerTimeOptions=&dynamicLayers=&gdbVersion=&mapScale=&f=image",
    local: lidarLocal + "dof84/{{record.BLOK}}/{{record.NAME}}.png"
}
var config = configDOF84;

var liberatorQueue = async.queue(liberatePair, 12);

parseDatabase(lidarLocal + "fishnet/LIDAR_FISHNET_D96.dbf");

function parseDatabase(file) {
    var parser = new Parser(file);

    parser.on('start', function(p) {
        console.log('Parsing...');
    });

    parser.on('header', function(h) {
        console.log("Parsed header");
    });

    parser.on('record', function(record) {
        if (record.CENTERX > minx && record.CENTERX < maxx &&
            record.CENTERY > miny && record.CENTERY < maxy) {
            found.push(record);
        }
    });

    parser.on('end', function(p) {
        console.log('Done');
        downloadNext();
    });

    parser.parse();
}

var index = 0;

function fileExists(path) {
    var filestat = null;
    try {
        filestat = fs.lstatSync(path);
    } catch (e) {}
    return !!(filestat && filestat.isFile());
}

// http://gis.arso.gov.si/lidar/gkot/b_35/D96TM/TM_462_101.zlas
// http://gis.arso.gov.si/lidar/otr/b_35/D96TM/TMR_462_101.zlas
// http://gis.arso.gov.si/arcgis/rest/services/opensource_dof84/MapServer/export?bbox=462000%2C101000%2C463000.00%2C102000.00&bboxSR=3794&layers=&layerDefs=&size=1000%2C1000&imageSR=3794&format=png&transparent=false&dpi=&time=&layerTimeOptions=&dynamicLayers=&gdbVersion=&mapScale=&f=image
function downloadNext() {
    // if (index > 0) return;
    if (index >= found.length) return;
    
    var record = found[index];
    index++;
    
    var params = {
        record: record,
        bounds: {
            "min": { "x": record.CENTERX - 500, "y": record.CENTERY - 500 },
            "max": { "x": record.CENTERX + 500, "y": record.CENTERY + 500 },
        },
        image: {
            width: 1000,
            height: 1000
        }
    }
    
    var local = Mustache.render(config.local, params);
    var filename = path.basename(local);
    
    console.log("processing", filename);
    
    if (fileExists(local)) {
        console.log("downloaded", filename);
        
        // Liberate zlas
        if (local.substr(-5) == ".zlas" && config.laz) {
            var laz = Mustache.render(config.laz, params);
            liberate(local, laz, downloadNext);
        } else {
            setTimeout(downloadNext, 0);
        }
    } else {
        mkdirp(path.dirname(local), function(err) {
            if (err) throw new Error(err);
            console.log("grabbing", filename);
            var remote = Mustache.render(config.remote, params);
            downloadFile(local, remote, downloadNext);
        });
    }
}

function liberate(local, laz, callback) {
    mkdirp(path.dirname(laz), function(err) {
        if (err) throw new Error("err");
        liberatorQueue.push({ source: local, drain: laz});
        callback();
    })
}

function liberatePair(pair, callback) {
    if (fileExists(pair.drain)) {
        console.log("liberated to", pair.drain);
        callback();
        return;
    }
    var cmd = liberator + " " + pair.source + " " + pair.drain;
    console.log("liberating to", pair.drain);
    exec(path.normalize(cmd), function(error, stdout, stderr) {
        if (error) console.error(error);
        callback();
    });
}

function downloadFile(local, remote, callback) {
    // console.log("downloading", remote, "to", local);
    var localTemp = local + ".part";
    var file = fs.createWriteStream(localTemp);
    var request = http.get(remote, function(response) {
        response.pipe(file);
        file.on('finish', function() {
            file.close(function closed() {
                fs.rename(localTemp, local, function renamed(err) {
                    callback();
                });
            });
        });
    }).on('error', function(err) {
        fs.unlink(dest);
        if (callback) callback(err.message);
    });
}