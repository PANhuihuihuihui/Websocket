var ws;

$(function() {
    ws = new WebSocket('ws://' + document.location.host + '/ws');
    ws.onopen = function() {
        console.log('on open');
    };
    ws.onclose = function() {
        console.log('on close');
    };
    ws.onmessage = function(msg) {
        if (evt.data != null) {
            var arrayBuffer = msg.data;
            var bytes = new Uint8Array(arrayBuffer);
            var image = document.getElementById('image');
            image.src = 'data:image/png;base64,'+encode(bytes);
        }
        ws.send();
    };

});
