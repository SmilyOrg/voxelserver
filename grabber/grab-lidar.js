var Parser = require('node-dbf');
var fs = require('fs');
var http = require('http');
var path = require('path');
var mkdirp = require('mkdirp');
var async = require('async');
var exec = require('child_process').exec;
var Mustache = require('mustache');

var ranges = [
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
        name: "Triglav",
        min: { x: 409844.18, y: 136916.37 },
        max: { x: 411717.43, y: 139003.93 },
    },
]

var found = [];

var size = 1000;
var hsize = size/2; 
var statusReportDelay = 500;

var queue = function(callback, length, noun, verbing, verbed) {
    var q = async.queue(callback, length);
    q.noun = noun;
    q.verbing = verbing;
    q.verbed = verbed;
    return q;
}.bind(this);

var initRecordQueue = queue(initRecord, 10);
var initResourceQueue = queue(initResource, 10);
var processResourceQueue = queue(processResource, 10);
var finishResourceQueue = queue(finishResource, 10);
var downloadQueue = queue(
    processResourceWithFunction.bind(null, downloadFile),
    2, "download", "downloading", "downloaded"
);
var liberatorQueue = queue(
    processResourceWithFunction.bind(null, liberateFile),
    6, "liberation", "liberating", "liberated"
);

// http://gis.arso.gov.si/lidar/gkot/b_35/D96TM/TM_462_101.zlas
// http://gis.arso.gov.si/lidar/otr/b_35/D96TM/TMR_462_101.zlas
// http://gis.arso.gov.si/arcgis/rest/services/opensource_dof84/MapServer/export?bbox=462000%2C101000%2C463000.00%2C102000.00&bboxSR=3794&layers=&layerDefs=&size=1000%2C1000&imageSR=3794&format=png&transparent=false&dpi=&time=&layerTimeOptions=&dynamicLayers=&gdbVersion=&mapScale=&f=image

var lidarRemote = "http://gis.arso.gov.si/";
var lidarLocal = "W:/gis/arso/";
var liberator = lidarLocal + "lasliberate/bin/lasliberate.exe";

var configs = [
    {
        name: "lidar laz",
        source: {
            name: "lidar zlas",
            source:
                lidarRemote +
                 "lidar/gkot/{{record.BLOK}}/D96TM/TM_{{record.NAME}}.zlas",
            drain:
                lidarLocal +
                "lidar/gkot/{{record.BLOK}}/D96TM/TM_{{record.NAME}}.zlas"
        },
        drain:
            lidarLocal +
            "laz/gkot/{{record.BLOK}}/D96TM/TM_{{record.NAME}}.laz",
        queue: liberatorQueue
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
]

var existingDrains = [];
var duplicated = 0;
var total = 0;

parseDatabase(lidarLocal + "fishnet/LIDAR_FISHNET_D96.dbf");



function parseDatabase(file) {
    var parser = new Parser(file);

    parser.on('start', function(p) {
        console.log('db parsing');
    });

    parser.on('header', function(h) {
        console.log("db header parsed");
    });

    parser.on('record', function(record) {
        ranges.forEach(function(range) {
            if (record.CENTERX > range.min.x-hsize && record.CENTERX < range.max.x+hsize &&
                record.CENTERY > range.min.y-hsize && record.CENTERY < range.max.y+hsize) {
                record.range = range;
                initRecordQueue.push(record);
            }
        }, this);
    });

    parser.on('end', function(p) {
        console.log('db parsed');
    });

    parser.parse();
}

function fileExists(path) {
    var filestat = null;
    try {
        filestat = fs.lstatSync(path);
    } catch (e) {}
    return !!(filestat && filestat.isFile());
}

function plog(resource) {
    console.log("  %s  %s  %s  %s",
        resource.record.range.name,
        resource.config.name,
        resource.name,
        Array.prototype.slice.call(arguments).slice(1).join("\t"));
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

    resource.params = {
        record: record,
        bounds: {
            "min": { "x": record.CENTERX - hsize, "y": record.CENTERY - hsize },
            "max": { "x": record.CENTERX + hsize, "y": record.CENTERY + hsize },
        },
        image: {
            width: size,
            height: size
        }
    }

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
        resource.source = "[unsourced]";
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
    if (resource.parent) {
        resource.parent.source = resource.drain;
        processResourceQueue.push(resource.parent);
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
            fs.unlink(drainTemp);
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