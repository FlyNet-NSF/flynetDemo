var latlonFactor = 1000;  // 100px per 0.1 lat or 0.1 lon

function getPixelsFromLat(lat) {
    var drone = $("#drone");

    var drone_lat = getDroneLat();

    var lat_delta = drone_lat - lat;
    var lat_pixels = lat_delta * latlonFactor;

    return lat_pixels;
}

function getPixelsFromLon(lon) {

}

function getDroneLat() {
    var drone_info = $("#drone_info .lat_input");
    return parseFloat(drone_info.text());
}

function getDroneLon() {
    var drone_info = $("#drone_info .lon_input");
    return parseFloat(drone_info.text());
}

function clearObjects() {
    $(".object").remove();
}

function setDroneHeading(heading) {
    var drone_info = $("#drone_info .head_input");
    drone_info.text(heading);

    var drone = $("#drone > img");
    drone.css("transform", "rotate(" + heading + "deg)")
}

function setDroneLat(lat) {
    var drone_info = $("#drone_info .lat_input");
    drone_info.text(lat);

    populateLatLines();
}

function setDroneLon(lon) {
    var drone_info = $("#drone_info .lon_input");
    drone_info.text(lon);
}

function populateLatLines() {
    var drone_lat = getDroneLat();
    var current_lat = (Math.round(drone_lat * 10) / 10) - 0.5;

    for (var i = 0; i <= 10; i++) {
        current_lat = Math.round(current_lat * 10) / 10;
        var pixels = getPixelsFromLat(current_lat);
        var new_line = "<div style='top: " + pixels + "px' class='lat_line map_line'><span>" + current_lat + "</span></div>";

        $("#drone").append(new_line);

        current_lat += 0.1;
    }
    getPixelsFromLat();
}

function addTower(heading) {

}

setInterval(function() { 
    $.getJSON('state.json', function(data) {
        clearObjects();
    
        $.each(data, function (key, val) {
            if (key == "drone") {
                $.each(data[key], function(key, val) {
                    switch(key) {
                        case "latitude":
                            setDroneLat(val);
                            break;
                        case "longitude":
                            setDroneLon(val);
                            break;
                        case "heading":
                            setDroneHeading(val)
                            break;
                    }
                });
            }
        });
    });
}, 5000);