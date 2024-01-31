from threading import Thread
import paho.mqtt.client as mqtt
import json
from time import time

class MQTTClient(Thread):
    def __init__(self, ip, port, topics = [], publish_topic="image_detection"):
        Thread.__init__(self)
        self.client = mqtt.Client("Demo_runner")
        self.topics = topics
        self.client.on_message = self.on_message
        self.client.on_connect = self.subscribe
        self.client.connect(ip, port)

        self.topic_infos = {}
        self.publish_topic = publish_topic
        self.message_ready = {key: False for key in topics}

        self.start()

    def on_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        self.topic_infos[msg.topic] = data
        self.message_ready[msg.topic] = True

    def is_message_ready(self, topic):
        if topic not in self.message_ready:
            self.message_ready[topic] = False
            return False
        return self.message_ready[topic]
                
    def subscribe(self, a, b, c, d):
        for topic in self.topics:
            self.client.subscribe(topic, qos=2)

    def publish(self, message):
        print("Published", message, "on", self.publish_topic)
        self.client.publish(self.publish_topic, json.dumps(message), qos=2)

    def publish_on_topic(self, topic, message, qos=2):
        print("Published", message, "on", topic)
        self.client.publish(topic, json.dumps(message), qos=qos)

    def run(self):
        self.client.loop_forever()

    def get_bb(self, topic):
        if topic not in self.topic_infos:
            return None
        res = self.topic_infos[topic]
        self.topic_infos[topic] = None
        self.message_ready[topic] = False
        return res

    def close(self):
        self.client.disconnect()