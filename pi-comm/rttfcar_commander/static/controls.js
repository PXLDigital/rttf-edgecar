//Connect to joy topic
var joyTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/master/joy',
    messageType : 'sensor_msgs/Joy'
});

//Create default joy message
var joy = new ROSLIB.Message({
    axes : [0, 0],
    buttons : [1]
});

//Connect to emergency break
var emergencyBreakTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/master/wheels_driver_node/emergency_stop',
    messageType : 'edgecar_msgs/BoolStamped'
});

//Create default emergency break
var emergencyBreak = new ROSLIB.Message({
    header: new Date(),
    data: true
});

//Generate buttons from config
for(let command of CONFIG.commands) {
    if(!command.disabled) {
        let command_button = $("<button>");
        command_button.text(command.title);
        command_button.click(function() {
            let shouldBreak = (command.title == "STOP") ? true : false;
            let emergency_msg = {
                // header:  new Date(),
                data: shouldBreak
            };
            console.log(emergency_msg)
            emergencyBreakTopic.publish(emergency_msg);
        });
        $("#command-buttons").append(command_button);
    }

    // if (command.roslib) {
    //     let command_button = $("<button>");
    //     command_button.text(command.title);
    //     command_button.click(function() {
    //         if (command.axes) {
    //             joy.axes = command.axes;
    //         }
    //         if (command.buttons) {
    //             joy.buttons = command.buttons
    //         }
    //     });
    //     $("#command-buttons").append(command_button);
    //}
}

//Create toggle button for publishing
let publish_button = $("<button id='publishButton'>");
publish_button.text(`Publish: ${publish}`);
publish_button.click(function() {
    publish = !publish;
    console.log(publish);
    document.getElementById("publishButton").innerHTML = `Publish: ${publish}`;
});
$("#command-buttons").append(publish_button);

//Publish every 100 ms
timer = setInterval( function () {
    if (publish) {
        joyTopic.publish(joy);    
    }            
}, 100)

//AI toggle
var aiCheckbox = document.querySelector("input[name=startAiCheckbox]");

window.onload = (event) => {
    toggleCamView(aiCheckbox.checked);
};

aiCheckbox.addEventListener('change', function() {                
    this.checked ? data = {"title": 'Start AI'} : data = {"title": 'Stop AI'};

    toggleCamView(this.checked);

    $.post( `${windowLocation}/execute`, data, function( result ) {
        $( ".result" ).html( result );
    });

});

//Connect to speed topic
var speedTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/master/speed',
    messageType : 'std_msgs/Float32'
});

//Create default speed message
var speedMessage = new ROSLIB.Message({
    data: .5
});

//Make speed slider work
speedSlider = document.getElementById("speed-slider");

speedSlider.addEventListener("change", function () {
    speedMessage.data = speedSlider.value * .01;
    speedTopic.publish(speedMessage);
});

//Create joystickcontroller
let myStick = new JoystickController("stick", 64, 8);
function update() {
    //document.getElementById("status").innerText = "Joystick: " + JSON.stringify(myStick.value);
    joy.axes = [myStick.value.y, clamp(myStick.value.x * 2, -1, 1)];
}

//Update joystick values
function loop() {
    requestAnimationFrame(loop);
    update();
}

function clamp(num, min, max) {
    return num <= min 
      ? min 
      : num >= max 
        ? max 
        : num
}

loop();