from flask import Flask, jsonify
from flask_socketio import SocketIO
import paho.mqtt.client as mqtt
import json
import logging
from gevent import monkey
monkey.patch_all()  # gevent için gerekli

# Log ayarları
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='gevent', logger=True, engineio_logger=True)

mqtt_broker = "localhost"
mqtt_port = 1883
mqtt_topics = ["factory/robot/odom", "factory/robot/map", "factory/robot/cmd_vel"]  # scan kaldırıldı

mqtt_client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    logger.info(f"Connected to MQTT Broker with code {rc}")
    for topic in mqtt_topics:
        client.subscribe(topic, qos=1)
        logger.debug(f"Subscribed to topic: {topic}")

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        logger.debug(f"Received MQTT message on {msg.topic}: {payload}")
        socketio.emit('mqtt_data', {'topic': msg.topic, 'payload': payload})
    except Exception as e:
        logger.error(f"Error processing MQTT message: {e}")

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

try:
    mqtt_client.connect(mqtt_broker, mqtt_port, keepalive=60)
    mqtt_client.loop_start()
except Exception as e:
    logger.error(f"Failed to connect to MQTT broker: {e}")

@app.route('/')
def index():
    return jsonify({"status": "Flask WebSocket Server Running"})

@socketio.on('publish_mqtt')
def handle_publish(data):
    topic = data.get('topic')
    message = data.get('message')
    logger.debug(f"Publishing to MQTT topic {topic}: {message}")
    mqtt_client.publish(topic, message, qos=1)

@socketio.on('connect')
def handle_connect():
    logger.info("Client connected to WebSocket")

@socketio.on('disconnect')
def handle_disconnect():
    logger.info("Client disconnected from WebSocket")

@socketio.on('connect_error')
def handle_connect_error(error):
    logger.error(f"WebSocket connection error: {error}")

if __name__ == '__main__':
    from geventwebsocket.handler import WebSocketHandler
    from gevent.pywsgi import WSGIServer
    http_server = WSGIServer(('0.0.0.0', 5000), app, handler_class=WebSocketHandler)
    logger.info("Starting WebSocket server on port 5000")
    http_server.serve_forever()