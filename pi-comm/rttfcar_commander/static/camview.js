//var cameraCheckbox = document.querySelector("input[name=cameraToggle]");
// cameraCheckbox.addEventListener('change', function() {
//     let data = {"stopCamera": 0};

//     if (this.checked) {
//         data = {"stopCamera": 0};
//     }
//     else {
//         document.getElementById('bg').src = "/static/img/no-signal.gif";
//         data = {"stopCamera": 1};
//     }              

//     $.post( `${windowLocation}/camera`, data, function( result ) {
//         $( ".result" ).html( result );
//     });

// });

//Connect to camera or detection topic
var image_topic = new ROSLIB.Topic({
    ros: ros, name: '/master/camera_node/image/compressed',
    messageType: 'sensor_msgs/CompressedImage'
});

var detection_topic = new ROSLIB.Topic({
    ros: ros, name: '/master/detector_node/image/compressed',
    messageType: 'sensor_msgs/CompressedImage'
});

//Display the right image if AI is enabled or not

function toggleCamView(checked) {
    if (checked) {
        image_topic.unsubscribe();
        detection_topic.subscribe(function(message) {
            document.getElementById('bg').src = "data:image/jpg;base64," + message.data;
        });
    } else {
        detection_topic.unsubscribe();
        image_topic.subscribe(function(message) {
            document.getElementById('bg').src = "data:image/jpg;base64," + message.data;
        });
    }
}