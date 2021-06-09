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
}

function setDroneLon(lon) {
    var drone_info = $("#drone_info .lon_input");
    drone_info.text(lon);
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