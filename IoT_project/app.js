decode = function(s) {
    var e={},i,b=0,c,x,l=0,a,r='',w=String.fromCharCode,L=s.length;
    var A="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for(i=0;i<64;i++){e[A.charAt(i)]=i;}
    for(x=0;x<L;x++){
        c=e[s.charAt(x)];b=(b<<6)+c;l+=6;
        while(l>=8){((a=(b>>>(l-=8))&0xff)||(x<(L-2)))&&(r+=w(a));}
    }
    return r;
};
var i = 1;
function ask(){
	i++;
	if (i >10 && i < 15){
		console.log("ask");
		if (i%2 == 0){
			ws.send("1");
		}
		else{ws.send("2");}
		
	}
	setTimeout(ask,500);

}

var ws;


    ws = new WebSocket('ws://' + document.location.host + '/IoT');
    ws.onopen = function() {
        console.log('on open');
    };
    ws.onclose = function() {
        console.log('on close');
    };
    ws.onmessage = function(msg) {
	console.log('on receive');
        if (msg.data != null) {
            var arrayBuffer = msg.data;
            console.log(arrayBuffer);
            image.src = "data:image/png;base64," + arrayBuffer;
        }
    };
    
    ask();


