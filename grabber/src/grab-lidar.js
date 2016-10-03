/// <reference path="../typings/index.d.ts" />

var DBFParser = require('node-dbf');
var fs = require('fs');
var http = require('http');
var path = require('path');
var mkdirp = require('mkdirp');
var async = require('async');
var exec = require('child_process').exec;
var Mustache = require('mustache');
var workerFarm = require('worker-farm');
var printf = require('printf');

var ranges = [
    // {
    //     name: "Ljubljana1",
    //     min: { x: 462000.00, y: 101000.00 },
    //     max: { x: 463000.00, y: 102000.00 },
    // },
    // {
    //     name: "Ljubljana9",
    //     min: { x: 462000.00, y: 101000.00 },
    //     max: { x: 465000.00, y: 104000.00 },
    // },
    {
        name: "Ljubljana",
        min: { x: 457262.46, y: 96189.57 },
        max: { x: 467601.05, y: 106686.92 },
    },
    {
        name: "Črnuče",
        min: { x: 462558.80, y: 104325.61 },
        max: { x: 467235.31, y: 108234.83 },
    },
    {
        name: "Bled",
        min: { x: 427984.52, y: 133178.09 },
        max: { x: 433977.33, y: 138449.91 },
    },
    {
        name: "Kranj",
        min: { x: 447247.90, y: 118731.50 },
        max: { x: 453928.63, y: 124922.75 },
    },
    {
        name: "Šmarna gora",
        min: { x: 454843.38, y: 104168.26 },
        max: { x: 461848.22, y: 113058.26 },
    },
    {
        name: "Piran",
        min: { x: 387600.49, y: 40519.20 },
        max: { x: 391324.50, y: 44931.13 },
    },
    {
        name: "Piran & Portorož",
        min: { x: 385000.00, y: 39000.00 },
        max: { x: 392000.00, y: 46000.00 },
    },
    {
        name: "Triglav",
        min: { x: 409844.18, y: 136916.37 },
        max: { x: 411717.43, y: 139003.93 },
    },
    {
        name: "Bohinj",
        min: { x: 407997.72, y: 124741.46 },
        max: { x: 416259.34, y: 130972.40 },
    },
    {
        name: "Maribor",
        min: { x: 539405.46, y: 149183.21 },
        max: { x: 560386.91, y: 164317.38 },
    }
];

var found = [];

var size = 1000;
var hsize = size/2; 
var statusReportDelay = 500;

var queues = [];
var queue = function(callback, length, noun, verbing, verbed) {
    var q = async.queue(callback, length);
    q.noun = noun;
    q.verbing = verbing;
    q.verbed = verbed;
    queues.push(q);
    return q;
}.bind(this);

var initRecordQueue = queue(initRecord, 10);
var initResourceQueue = queue(initResource, 10);
var processResourceQueue = queue(processResource, 10);
var finishResourceQueue = queue(finishResource, 10);
var downloadQueue = queue(
    processResourceWithFunction.bind(null, downloadFile),
    4, "download", "downloading", "downloaded"
);
var liberatorQueue = queue(
    processResourceWithFunction.bind(null, liberateFile),
    6, "liberation", "liberating", "liberated"
);
var dmrQueue = queue(
    processDMRFile,
    12, "dmr compression", "dmr compressing", "dmr compressed"
);

// http://gis.arso.gov.si/lidar/gkot/b_35/D96TM/TM_462_101.zlas
// http://gis.arso.gov.si/lidar/otr/b_35/D96TM/TMR_462_101.zlas
// http://gis.arso.gov.si/arcgis/rest/services/opensource_dof84/MapServer/export?bbox=462000%2C101000%2C463000.00%2C102000.00&bboxSR=3794&layers=&layerDefs=&size=1000%2C1000&imageSR=3794&format=png&transparent=false&dpi=&time=&layerTimeOptions=&dynamicLayers=&gdbVersion=&mapScale=&f=image
// http://gis.arso.gov.si/lidar/dmr1/b_35/D96TM/TM1_462_102.asc

var lidarRemote = "http://gis.arso.gov.si/";
var lidarLocal = "W:/gis/arso/";
var lidarLocalMass = "D:/gis/arso/";
var liberator = lidarLocal + "lasliberate/bin/lasliberate.exe";

var configs = [
    {
        name: "laz",
        source: {
            name: "zlas",
            source:
                lidarRemote +
                "lidar/gkot/{{record.BLOK}}/D96TM/TM_{{record.NAME}}.zlas",
            drain:
                lidarLocalMass +
                "lidar/gkot/{{record.BLOK}}/D96TM/TM_{{record.NAME}}.zlas"
        },
        drain:
            lidarLocal +
            "laz/gkot/{{record.BLOK}}/D96TM/TM_{{record.NAME}}.laz",
        queue: liberatorQueue,
        deleteSource: true
    },
    {
        name: "bdmr",
        source: {
            name: "dmr",
            source:
                lidarRemote +
                "lidar/dmr1/{{record.BLOK}}/D96TM/TM1_{{record.NAME}}.asc",
            drain:
                lidarLocalMass +
                "lidar/dmr1/{{record.BLOK}}/D96TM/TM1_{{record.NAME}}.asc",
        },
        drain:
            lidarLocal +
            "bdmr/{{record.BLOK}}/D96TM/TM1_{{record.NAME}}.bin",
        queue: dmrQueue
    },
    {
        name: "map",
        source:
            lidarRemote +
            "arcgis/rest/services/opensource_dof84/MapServer/export" +
            "?bbox="+
            "{{bounds.min.x}},{{bounds.min.y}}," +
            "{{bounds.max.x}},{{bounds.max.y}}" + 
            "&size={{image.width}},{{image.height}}" + 
            "&bboxSR=3794&imageSR=3794" + 
            "&format=png&f=image",
        drain:
            lidarLocal +
            "dof84/{{record.BLOK}}/{{record.NAME}}.png"
    }
];

var existingDrains = [];
var duplicated = 0;
var total = 0;

var bdmrWorkers = workerFarm(require.resolve('./bdmr'));
var dbfParsed = false;

parseDatabase(lidarLocal + "fishnet/LIDAR_FISHNET_D96.dbf");



function parseDatabase(file) {
    var dbfParser = new DBFParser(file);

    dbfParser.on('start', function(p) {
        // console.log('db parsing');
    });

    dbfParser.on('header', function(h) {
        // console.log("db header parsed");
        
    });

    dbfParser.on('record', function(record) {
        ranges.forEach(function(range) {
            if (record.CENTERX > range.min.x-hsize && record.CENTERX < range.max.x+hsize &&
                record.CENTERY > range.min.y-hsize && record.CENTERY < range.max.y+hsize) {
                record.range = range;
                initRecordQueue.push(record);
            }
        }, this);
    });

    dbfParser.on('end', function(p) {
        // console.log('db parsed');
        dbfParsed = true;
    });

    dbfParser.parse();
}

function fileExists(path) {
    var filestat = null;
    try {
        filestat = fs.lstatSync(path);
    } catch (e) {}
    return !!(filestat && filestat.isFile());
}

function getTotalQueueLength() {
    var qlen = queues.reduce(function(a, b) {
        return a + b.length();
    }, 0);
    if (!dbfParsed) qlen++;
    return qlen;
}

function plog(resource) {

    var qlens = queues.reduce(function(a, b) {
        return a + printf("%4d", b.length());
    }, "");

    console.log(printf("%s %12s %5s %16s  %s",
        qlens,
        resource.record.range.name,
        resource.config.name,
        resource.name,
        Array.prototype.slice.call(arguments).slice(1).join("\t")
    ));
    // console.log("  %s  %s  %s  %s",
    //     resource.record.range.name,
    //     resource.config.name,
    //     resource.name,
    //     Array.prototype.slice.call(arguments).slice(1).join("\t"));
}

function initRecord(record, done) {
    configs.forEach(function(config) {
        initResourceQueue.push({
            record: record,
            config: config
        });
    }, this);
    done();
}

function initResource(resource, done) {
    
    var record = resource.record;
    var config = resource.config;

    var nameSpl = resource.record.NAME.split("_", 2);
    if (nameSpl.length != 2) {
        console.error("Unable to split name to coordinates: " + nameSpl);
        done();
        return;
    }

    var coords = nameSpl.map(function(coord) { return Number(coord)*1000; });

    resource.params = {
        record: record,
        coords: coords,
        bounds: {
            "min": { "x": coords[0], "y": coords[1] },
            "max": { "x": coords[0] + size, "y": coords[1] + size },
            // "min": { "x": record.CENTERX - hsize, "y": record.CENTERY - hsize },
            // "max": { "x": record.CENTERX + hsize, "y": record.CENTERY + hsize },
        },
        image: {
            width: size,
            height: size
        }
    };

    total++;

    resource.drain = Mustache.render(config.drain, resource.params);
    resource.name = path.basename(resource.drain);
    if (existingDrains.indexOf(resource.drain) != -1) {
        duplicated++;
        done();
        return;
    }
    existingDrains.push(resource.drain);

    // plog(resource, "processing");

    if (fileExists(resource.drain)) {
        resource.source = "";
        finishResourceQueue.push(resource);
        done();
    } else {
        mkdirp(path.dirname(resource.drain), function(err) {
            if (err) throw new Error(err);
            
            if (typeof config.source == "string") {
                // URL to file
                resource.source = Mustache.render(config.source, resource.params);
                processResourceQueue.push(resource);
            } else {
                // Subconfig
                initResourceQueue.push({
                    record: resource.record,
                    config: resource.config.source,
                    parent: resource
                });
            }

            done();
        });
    }

}

function processResource(resource, done) {
    
    var config = resource.config;
    var name = resource.name;

    if (resource.source) {
        if (!config.queue) config.queue = downloadQueue;
        plog(resource, config.queue.noun + " required");
        resource.startTime = Date.now();
        config.queue.push(resource);
    } else {
        plog(resource, "unable to source");
    }

    done();
}

function finishResource(resource, done) {

    var config = resource.config;
    var name = resource.name;

    if (Date.now() - resource.startTime > statusReportDelay) {
        plog(resource, config.queue.verbed);
    }

    // Delete source if configured
    if (config.deleteSource && resource.source !== "" && fileExists(resource.source)) {
        var deleteName = resource.source;
        var lastSlash = resource.source.lastIndexOf("/");
        if (lastSlash >= 0) deleteName = deleteName.substr(lastSlash + 1);
        plog(resource, "deleting " + deleteName);
        fs.unlink(resource.source);
    }

    if (resource.parent) {
        resource.parent.source = resource.drain;
        processResourceQueue.push(resource.parent);
    }

    if (getTotalQueueLength() === 0) {
        workerFarm.end(bdmrWorkers);
    }

    done();
}

function processResourceWithFunction(func, resource, done) {
    plog(resource, resource.config.queue.verbing);
    func(resource.source, resource.drain, function onFinish(err) {
        if (err) {
            plog(resource, resource.config.queue.noun + " error: " + error);
        } else {
            finishResourceQueue.push(resource);
        }
        done();
    });
}

function downloadFile(remote, local, done) {
    var localTemp = local + ".part";
    var file = fs.createWriteStream(localTemp);
    var request = http.get(remote, function(response) {
        file.on('finish', function() {
            file.close(function closed() {
                fs.rename(localTemp, local, function renamed(err) {
                    if (done) done();
                });
            });
        });
        response.pipe(file);
    }).on('error', function(err) {
        fs.unlink(localTemp);
        if (done) done(err.message);
    });
}

function liberateFile(zlas, drain, done) {
    var indexExt = ".lax";
    
    var ext = path.extname(drain);
    var dir = path.dirname(drain);
    var base = path.basename(drain, ext);
    
    var index = path.join(dir, base + indexExt);
    
    var prefix = base + ".part";
    var drainTemp = path.join(dir, prefix + ext);
    var indexTemp = path.join(dir, prefix + indexExt);
    
    var cmd = liberator + " " + zlas + " " + drainTemp;
    exec(path.normalize(cmd), function(error, stdout, stderr) {
        if (error) {
            console.error("Liberation error: " + stderr);
            if (fileExists(drainTemp)) fs.unlink(drainTemp);
            if (done) done();
        } else {
            async.parallel([
                function renameIndex(callback) {
                    fs.rename(indexTemp, index, function renamedIndex(err) { callback(); });  
                },
                function renameDrain(callback) {
                    fs.rename(drainTemp, drain, function renamedDrain(err) { callback(); });
                }
            ], done);
        }
    });
}

function processDMRFile(resource, done) {
    plog(resource, resource.config.queue.verbing);
    var source = resource.source;
    var drain = resource.drain;
    var coords = resource.params.coords;

    var ext = path.extname(drain);
    var dir = path.dirname(drain);
    var base = path.basename(drain, ext);
    
    var prefix = base + ".part";
    var drainTemp = path.join(dir, prefix + ext);
    
    bdmrWorkers({
        source: source,
        drainTemp: drainTemp,
        drain: drain,
        coords: coords
    }, function (err, output) {
        if (err) console.error("bdmr worker execution error: " + err);
        if (output.err) console.error("bdmr worker error: " + output.err);
        finishResourceQueue.push(resource);
        done();
    });
}

/*
function downloadResource(resource, done) {
    plog(resource, resource.config.queue.verbing);
    var name = resource.name;
    downloadFile(resource.source, resource.drain, function onFinish(err) {
        if (err) {
            plog(resource, "download error: " + err);
        } else {
            finishResourceQueue.push(resource);
        }
        done();
    });
}

function liberateResource(resource, done) {
    plog(resource, resource.config.queue.verbing);
    liberateFile(resource.source, resource.drain, function onFinish(err) {
        if (err) {
            plog(resource, "liberation error: " + error);
        } else {
            finishResourceQueue.push(resource);
        }
        done();
    });
}
*/