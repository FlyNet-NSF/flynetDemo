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

function addGroundStation(station_lat, station_lon) {
    station_width_pxl = getPixelsFromLat(station_lat, drone_lat_last) * -1;
    station_height_pxl = getPixelsFromLon(station_lon, drone_lon_last) * -1;

    var tower = "<div style='left: " + station_width_pxl + "; top: " + station_height_pxl + ";' class='ctower clearable'><img src='assets/station.png'></div>"

    addDroneObject(tower);
}

function addPath(source_lat, source_lon, dest_lat, dest_lon) {
    source_width_pxl = getPixelsFromLat(source_lat);
    source_height_pxl = getPixelsFromLon(source_lon);
    dest_width_pxl = getPixelsFromLat(dest_lat);
    dest_height_pxl = getPixelsFromLon(dest_lon);

    var svg_width = Math.abs(dest_width_pxl - source_width_pxl);
    var svg_height = Math.abs(dest_height_pxl - source_height_pxl);

    var x1 = 0;
    var x2 = 0;
    var y1 = 0;
    var y2 = 0;

    if (source_width_pxl > dest_width_pxl) {
        x1 = 0;
        x2 = svg_width;
    } else {
        x1 = svg_width;
        x2 = 0;
    }

    if (source_height_pxl > dest_height_pxl) {
        y1 = svg_height;
        y2 = 0;
    } else {
        y1 = 0;
        y2 = svg_height;
    }

    var svg_script = "<svg width='" + svg_width + " height='" + svg_height + "''>";
    svg_script += "<line x1='" + x1 + "' y1='" + y1 + "' x2='" + x2 + "' y2='" + y2 + "' style='stroke:rgb(255,0,0);stroke-width:2' />";
    svg_script += "</svg>";

    addDroneObject(svg_script);
}

setInterval(function() { 
    $.ajax({cache: false, success: function(data) {
        clearObjects();
    
        $.each(data, function (key, val) {
            if (key == "drone") {
                // drone data (including cell towers)
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
            } else if (key == "stations") {
                // ground stations
                $.each(data[key], function(key, val) {
                    addGroundStation(val["latitude"], val["longitude"]);
                });
            } else if (key == "weights") {
                // generated graph data
                $.each(data[key], function(key, val) {
                    
                });
            }
        });
    }, url: 'state.json'});
}, 2000);