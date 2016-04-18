var d3stats;

function loadStats() {
    $.getJSON("stats.json")
        .done(function( data ) {
            var stats = d3stats.selectAll(".statistic").data(data)
            
            // console.log(stats);
            
            var stat = stats.enter().append("div")
                .attr("class", "statistic")
            
            stat.append("div")
                .attr("class", "value")
            
            stat.append("div")
                .attr("class", "label")
            
            stats.exit().remove();
            
            stats.select(".value")
                .text(function(data) { return data.value.toLocaleString(); })
                
            stats.select(".label")
                .text(function(data) { return data.name; })
            
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