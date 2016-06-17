var d3stats;
var d3execTimeList;
var d3execTimeBars;
var execTimeSeries = {};
var execTimeSeriesCount = 0;
var colors;
var barChart;
var excluded = [];

var STAT = 0;
var EXEC_TIME = 1;


function formatMicro(v) {
    return (v*1e-3).toFixed(2) + "ms";
}

function updateCharts(raw) {
    
    dataTime = raw.filter(function(counter) {
        return counter.type == EXEC_TIME;
    });

    dataTimeExcl = dataTime.filter(function(counter) {
        return excluded.indexOf(counter.name) == -1; 
    })
    
    var time = new Date().getTime();
    var stacked = 0;
    dataTimeExcl.forEach(function(counter) {
        var ts = execTimeSeries[counter.id];
        if (!ts) {
            series = new TimeSeries();
            ts = execTimeSeries[counter.id] = { index: execTimeSeriesCount, series: series };
            chart.addTimeSeries(series, { lineWidth: 4, strokeStyle: colors(ts.index) });
            execTimeSeriesCount++;
        }
        var value = counter.value*1e-3;
        stacked += value;
        ts.series.append(time, value);
    }, this);
    
    d3execTimeBars
        .datum([{ values: dataTimeExcl }])
        .call(barChart)
    
    
    var list = d3execTimeList
        .selectAll(".item")
        .data(dataTime)
        
    var item = list
        .enter()
        .append("div")
        .attr("class", "ui checkbox item")
    
    /*
    label.append("span")
        .attr("class", "name")
    
    label.append("div")
        .attr("class", "detail")
    */

    item.append("input")
        .attr("type", "checkbox")
        .each(function(counter) {
            if (excluded.indexOf(counter.name) == -1) d3.select(this).attr("checked", "checked");
        })

    var label = item.append("label")
        .attr("class", "")
        .append("div")
        .attr("class", "ui inverted label")

    label.append("span")
        .attr("class", "name")
    
    label.append("div")
        .attr("class", "detail")

    list.exit().remove()
    
    $(item[0].filter(function(obj) { return obj != null; }))
        .checkbox({
            onChange: function() {
                var check = $(this).parent();
                var checked = check.checkbox("is checked");
                var name = check.find(".name").text();
                var index = excluded.indexOf(name);
                if (checked) {
                    if (index != -1) excluded.splice(index, 1);
                } else {
                    if (index == -1) excluded.push(name);
                }
            }
        })
    ;

    var labels = list.select(".label")

    labels
        .style("background-color", function(counter) { return colors(execTimeSeries[counter.id].index) })
    
    labels.select(".name")
        .text(function(counter) { return counter.name })
        
    labels.select(".detail")
        .text(function(counter) { return formatMicro(counter.value); })
    
}

function updateStats(data) {
    
    data = data.filter(function(counter) { return counter.type == STAT; });
    
    var stats = d3stats
        .selectAll(".statistic")
        .data(data)
    
    var stat = stats
        .enter()
        .append("div")
        .attr("class", "ui statistic")
    
    stat.append("div")
        .attr("class", "value")
    
    stat.append("div")
        .attr("class", "label")
    
    stats.exit().remove();
    
    stats.select(".value")
        .text(function(counter) { return counter.value.toLocaleString(); })
        
    stats.select(".label")
        .text(function(counter) { return counter.name; })
}

function getCounter(data, id) {
    var filtered = data.filter(function(counter) { return counter.id == id; });
    if (filtered.length == 0) return null;
    return filtered[0];
}


function loadStats() {
    $.getJSON("stats.json")
        .done(function( data ) {
            
            var created = getCounter(data, "boxesCreated");
            var generation = getCounter(data, "box generation");

            if (generation) {
                data.push({
                    type: STAT,
                    id: "generationPerCreated",
                    name: "Box generation time [ms]",
                    value: created.value > 0 ? Math.round(generation.value / created.value * 1e-3) : 0
                })
            }
            
            updateStats(data);
            updateCharts(data);
            
            setTimeout(loadStats, 1000);
        })
        .fail(function( jqxhr, textStatus, error ) {
            setTimeout(loadStats, 2000);
        });
}


$(document).ready(function() {
    var $boxes = $("#boxes");
    var $mapClouds = $("#mapClouds");
    setInterval(function() {
        $boxes.attr("src", "boxes.png?"+(new Date().getTime()));
        $mapClouds.attr("src", "mapClouds.png?"+(new Date().getTime()));
    }, 1000);
    
    d3stats = d3.select("#stats")
    d3execTimeList = d3.select("#execTimeList")
    d3execTimeBars = d3.select('#execTimeBars svg')
    
    chart = new SmoothieChart( {
        grid: { fillStyle:'transparent', strokeStyle: '#aaaaaa' },
        labels: { fillStyle: '#000000' },
    } );
    var canvas = document.getElementById('execTime');
    chart.streamTo(canvas, 1200);
    
    colors = d3.scale.category20b();
    
    function resizeSmoothie() {
        $(canvas).attr("width", $(canvas).parent().width());
    }
    nv.utils.windowResize(resizeSmoothie); resizeSmoothie();
    
    nv.addGraph(function() {
        barChart = nv.models.multiBarHorizontalChart()
            .x(function(d) { return d.name })
            .y(function(d) { return d.value })
            .barColor(colors.range())
            .duration(250)
            .margin({left: 120})
            .showControls(false)
            .showLegend(false)
            // .y(function(d) { console.log(d); return d; })
        nv.utils.windowResize(barChart.update);
        return barChart;
    });
    
    
    // colors = d3.scale.ordinal().range(
        // ["#a6cee3","#1f78b4","#b2df8a","#33a02c","#fb9a99","#e31a1c","#fdbf6f","#ff7f00","#cab2d6","#6a3d9a","#e7f593","#b15928"]
        // ["#8dd3c7","#ffffb3","#bebada","#fb8072","#80b1d3","#fdb462","#b3de69","#fccde5","#d9d9d9","#bc80bd","#ccebc5","#ffed6f"]
        // ["#e41a1c","#377eb8","#4daf4a","#984ea3","#ff7f00","#ffff33","#a65628","#f781bf","#999999"]
    // ) 
    
    /*
    
            <div class="statistic">
                <div class="value">
                22
                </div>
                <div class="label">
                Faves
                </div>
            </div>
            <div class="statistic">
                <div class="value">
                31,200
                </div>
                <div class="label">
                Views
                </div>
            </div>
            <div class="statistic">
                <div class="value">
                22
                </div>
                <div class="label">
                Members
                </div>
            </div>
    */
    
    loadStats();
});