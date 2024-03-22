const mqtt = require("mqtt");
const rest = require("restler");
const ifttt_event_key = "jjNOhqJ1_fgJ9FV3KE_G4W-uB9rBetCStQHoIhkLuhS";
const url = "mqtt://192.168.1.184";
//const url = 'mqtt://192.168.145.240'

const options = {
    port: 1883,
    host: url,
    clientId: "drone_" + Math.random().toString(16).substr(2, 8),
    username: "guest",
    password: "guest",
};

function sendFeedbackMqtt(q, msg) {
    const client = mqtt.connect(url, options);
    client.on("connect", function() {
        client.publish(q, msg, { qos: 2 }, function() {
            client.end();
        });
    });
}

exports.handler = function(context, event) {
    var obj = JSON.parse(event.body);
    if (obj.sensor == "FIRE_OFF") {
        rest
            .post(
                "https://maker.ifttt.com/trigger/fire_notification/with/key/" +
                ifttt_event_key, {
                    data: {
                        value1: obj.sensor,
                        value2: obj.latitude,
                        value3: obj.longitude,
                    },
                }
            )
            .on("complete", function(data) {
                console.log(
                    "Forest status: " +
                    obj.sensor +
                    "Latitude: " +
                    obj.latitude +
                    "Longitude: " +
                    obj.longitude
                );
            });
    }
    context.callback("");
};