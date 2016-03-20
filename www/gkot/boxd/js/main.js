$(document).ready(function() {
    var $boxes = $("#boxes");
    setInterval(function() {
        $boxes.attr("src", "boxes.png?"+(new Date().getTime()));
    }, 1000);
});