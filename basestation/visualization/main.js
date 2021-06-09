$.getJSON('../../data/file.json', function(data) {         
    alert(data);

    setTimeout(arguments.callee, 10000);
});