var latlonFactor = 3000;  // 100px per 0.1 lat or 0.1 lon, also determines initial zoom
var svg_size = 1000;  // predefined SVG width

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

    var drone_wrapper = $("#drone_icon");
    drone_wrapper.attr("data-heading", heading);
}

var drone_lat_last = 0;
function setDroneLat(lat) {
    drone_lat_last = lat;

    var drone_info = $("#drone_info .lat_input");
    drone_info.text(lat);

    var drone_wrapper = $("#drone_icon");
    drone_wrapper.attr("data-lat", lat);

    populateLatLines(lat);
}

var drone_lon_last = 0;
function setDroneLon(lon) {
    drone_lon_last = lon;
    var drone_info = $("#drone_info .lon_input");
    drone_info.text(lon);

    var drone_wrapper = $("#drone_icon");
    drone_wrapper.attr("data-lon", lon);

    populateLonLines(lon);
}

function addDroneObject(object) {
    $("#drone").append(object)
}

function addSVGObject(object) {
    $("#drone > svg").append(object);
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

function addTower(tower_lat, tower_lon, id) {
    tower_width_pxl = getPixelsFromLat(tower_lat, drone_lat_last);
    tower_height_pxl = getPixelsFromLon(tower_lon, drone_lon_last);

    var tower = "<div data-id='" + id + "' data-lat='" + tower_lat + "' data-lon='" + tower_lon + "' style='left: " + tower_width_pxl + "; top: " + tower_height_pxl + ";' class='ctower clearable'><img src='assets/tower.png'></div>"

    addDroneObject(tower);
}

function addGroundStation(station_lat, station_lon, id) {
    station_width_pxl = getPixelsFromLat(station_lat, drone_lat_last);
    station_height_pxl = getPixelsFromLon(station_lon, drone_lon_last);

    var station = "<div data-id='" + id + "' data-lat='" + station_lat + "' data-lon='" + station_lon + "' style='left: " + station_width_pxl + "; top: " + station_height_pxl + ";' class='gstation clearable'><img src='assets/station.png'></div>"

    addDroneObject(station);
}

function addPathByID(source_id, dest_id) {
    var source_elem = $("div").find(`[data-id='${source_id}']`);
    var dest_elem = $("div").find(`[data-id='${dest_id}']`);

    if (source_id == "drone") {
        source_elem = $("#drone_icon");
    }

    if (dest_id == "drone") {
        dest_elem = $("#drone_icon");
    }

    var source_lat = source_elem.attr("data-lat");
    var source_lon = source_elem.attr("data-lon");
    var dest_lat = dest_elem.attr("data-lat");
    var dest_lon = dest_elem.attr("data-lon");

    addPath(source_lat, source_lon, dest_lat, dest_lon);
}

function addPath(source_lat, source_lon, dest_lat, dest_lon) {
    source_width_pxl = getPixelsFromLat(source_lat, drone_lat_last);
    source_height_pxl = getPixelsFromLon(source_lon, drone_lon_last);
    dest_width_pxl = getPixelsFromLat(dest_lat, drone_lat_last);
    dest_height_pxl = getPixelsFromLon(dest_lon, drone_lon_last);

    let x1 = Math.round(source_width_pxl + (svg_size / 2));
    let y1 = Math.round(source_height_pxl + (svg_size / 2));
    let x2 = Math.round(dest_width_pxl + (svg_size / 2));
    let y2 = Math.round(dest_height_pxl + (svg_size / 2));

    var svg_script = "<line class='clearable' x1='" + x1 + "' x2='" + x2 + "' y1='" + y1 + "' y2='" + y2 + "' stroke='#5AB1BB' stroke-width='2'></line>";
    // NEED TO FIX NAMESPACE HERE
    addSVGObject(svg_script);
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
                                addTower(val["latitude"], val["longitude"], val["id"]);
                            });
                    }
                });
            } else if (key == "stations") {
                // ground stations
                $.each(data[key], function(key, val) {
                    addGroundStation(val["latitude"], val["longitude"], val["id"]);
                });
            } else if (key == "weights") {
                // generated graph data
                $.each(data[key], function(key, val) {
                    source_key = key;
                    $.each(data["weights"][key], function(key, val) {
                        addPathByID(source_key, val[0]);
                    });
                });
            }

            $("body").html($("body").html());  // refresh SVG
        });
    }, url: 'state.json'});
}, 5000);