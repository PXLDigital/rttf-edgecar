const windowLocation = window.location.href;
const port = 5000;
var publish = false;

//Create ros connection
var ros = new ROSLIB.Ros({
    url : `ws://${window.location.hostname}:9090`
});