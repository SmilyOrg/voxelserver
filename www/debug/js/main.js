// var origin = {
//     x: 0,
//     z: -1000,
//     y: 0
// }

// var origin = {
//     x: 462000,
//     z: 101000,
//     y: 0
// }

var params = {
    
    ox: 0,
    oz: -128*0,
    oy: 0,
    
    sx: 128, sz: 128,
    // sx: 256, sz: 256,
    sy: 256,
    
    gw: 8,
    gh: 8,
    
    // type: "lidar",
    
};

function getTilesHTML(params) {
    var ox = params.ox;
    var oy = params.oy;
    var oz = params.oz;
    
    var sx = params.sx;
    var sy = params.sy;
    var sz = params.sz;
    
    var gw = params.gw;
    var gh = params.gh;
    
    var type = params.type;
    
    function getTileImage(type, x, y, z) {
        return "tile_" + type + "_" + sx + "_" + sy + "_" + sz + "_" + x + "_" + y + "_" + z + ".png";
    }
    
    var html = "";
    for (var iy = 0; iy < gh; iy++) {
        html += "<tr>";
        for (var ix = 0; ix < gw; ix++) {
            var x = ox + ix * sx;
            var y = oy + 0;
            var z = oz + iy * sz;
            var info = x + ", " + y + ", " + z;
            html += "<td>";
            html +=     "<div class='ui gray label'>" + info + "</div>";
            if (type == "overlay") {
                var lidar = getTileImage("lidar", x, y, z);
                var map = getTileImage("map", x, y, z);
                html += "<img class=\"map\" width=\"" + sx + "\" height=\"" + sz + "\" alt=\"map\" src=\"" + map + "\" />";
                html += "<img class=\"lidar\" width=\"" + sx + "\" height=\"" + sz + "\" alt=\"lidar\" src=\"" + lidar + "\" />";  
            } else {
                var tile = getTileImage(type, x, y, z);
                html += "<img width=\"" + sx + "\" height=\"" + sz + "\" alt=\"tile\" src=\"" + tile + "\" />";
            }
            html += "</td>";
        }
        html += "</tr>";
    }
    
    return html;
}

function updateTiles(params) {
    // Move to origin
    // params.ox += Math.floor(origin.x/params.sx)*params.sx;
    // params.oz += Math.floor(origin.z/params.sz)*params.sz;
    // params.oy += origin.y;

    // Center
    params.ox += -params.sx*Math.floor(params.gw/2);
    params.oz += -params.sz*Math.floor(params.gh/2);
    
    params.type = "lidar";
    $("#tiles-lidar").html(getTilesHTML(params));
    
    // params.type = "map";
    // $("#tiles-map").html(getTilesHTML(params));
    
    // params.type = "overlay";
    // $("#tiles-map").html(getTilesHTML(params));
}

$(document).ready(function() {
    
    updateTiles(params);
    
});