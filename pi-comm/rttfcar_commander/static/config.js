let CONFIG = {
    "default_ip": "172.16.104.77", //"192.168.43.171",
    "commands": [
        {
            "command": "/home/user/scripts/start_ai.sh",
            "title": "Start AI",
            "disabled": true
        },

        {
            "command": "/home/user/scripts/stop_ai.sh",
            "title": "Stop AI",
            "disabled": true
        },

        {
            "command": "docker exec rttf-edgecar_cardrivers_1 /home/cardrivers/start_recording.sh",
            "title": "Start REC",
            "disabled": true
        },

        {
            "command": "docker exec rttf-edgecar_cardrivers_1 /home/cardrivers/stop_recording.sh",
            "title": "Stop REC",
            "disabled": true
        },

        {
            "command": "docker exec rttf-edgecar_cardrivers_1 ls",
            "title": "LS HOME",
            "disabled": true
        },
        {
            "title": "START",
            "roslib": true,
            "buttons": [1]
        },

        {
            "title": "STOP",
            "roslib": true,
            "buttons": [0]
        },
    ]
}
