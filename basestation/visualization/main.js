var latlonFactor = 3000;  // 100px per 0.1 lat or 0.1 lon

function getPixelsFromLat(lat, drone_lat) {
    var lat_delta = drone_lat - lat;
    var lat_pixels = lat_delta * latlonFactor;

    return lat_pixels;
}

function getPixelsFromLon(lon, drone_lon) {
    var lon_delta = drone_lon - lon;
    var lon_pixels = lon_delta * latlonFactor * -1;

    return lon_pixels;
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
    $(".clearable").remove();
}

function setDroneHeading(heading) {
    var drone_info = $("#drone_info .head_input");
    drone_info.text(heading);

    var drone = $("#drone_icon > img");
    drone.css("transform", "rotate(" + heading + "deg)")
}

var drone_lat_last = 0;
function setDroneLat(lat) {
    drone_lat_last = lat;

    var drone_info = $("#drone_info .lat_input");
    drone_info.text(lat);

    populateLatLines(lat);
}

var drone_lon_last = 0;
function setDroneLon(lon) {
    drone_lon_last = lon;
    var drone_info = $("#drone_info .lon_input");
    drone_info.text(lon);

    populateLonLines(lon);
}

function addDroneObject(object) {
    $("#drone").append(object)
}

function populateLatLines(drone_lat) {
    var current_lat = (Math.round(drone_lat * 10) / 10) - 0.5;

    for (var i = 0; i <= 10; i++) {
        current_lat = Math.round(current_lat * 10) / 10;
        var pixels = getPixelsFromLat(current_lat, drone_lat);
        var new_line = "<div style='top: " + pixels + "px' class='lat_line clearable'><span>" + current_lat + "</span></div>";

        addDroneObject(new_line);

        current_lat += 0.1;
    }
}

function populateLonLines(drone_lon) {
    var current_lon = (Math.round(drone_lon * 10) / 10) - 0.5;

    for (var i = 0; i <= 10; i++) {
        current_lon = Math.round(current_lon * 10) / 10;
        var pixels = getPixelsFromLon(current_lon, drone_lon);
        var new_line = "<div style='left: " + pixels + "px' class='lon_line clearable'><span>" + current_lon + "</span></div>";

        addDroneObject(new_line);

        current_lon += 0.1;
    }
}

function addTower(tower_lat, tower_lon) {
    tower_width_pxl = getPixelsFromLat(tower_lat, drone_lat_last) * -1;
    tower_height_pxl = getPixelsFromLon(tower_lon, drone_lon_last) * -1;

    var tower = "<div style='left: " + tower_width_pxl + "; top: " + tower_height_pxl + ";' class='ctower clearable'><img src='assets/tower.png'></div>"

    addDroneObject(tower);
}

setInterval(function() { 
    $.ajax({cache: false, success: function(data) {
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
                        case "celltowers":
                            $.each(data["drone"][key], function(key, val) {
                                addTower(val["latitude"], val["longitude"]);
                            });
                    }
                });
            }
        });
    }, url: 'state.json'});
}, 2000);